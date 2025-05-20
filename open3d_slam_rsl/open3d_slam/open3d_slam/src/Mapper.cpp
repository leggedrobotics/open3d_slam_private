/*
 * Mapper.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#include "open3d_slam/Mapper.hpp"
#include "open3d_slam/ScanToMapRegistration.hpp"
#include "open3d_slam/Voxel.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/time.hpp"

#include "open3d_conversions/open3d_conversions.h"

#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/utility/Eigen.h>
#include <open3d/utility/Helper.h>
#include "open3d_slam/ProfilerScopeGuard.hpp"

#pragma omp declare reduction(vec3d_plus : Eigen::Vector3d : omp_out += omp_in) initializer(omp_priv = Eigen::Vector3d::Zero())

namespace o3d_slam {

namespace {
namespace registration = open3d::pipelines::registration;
const bool isCheckTransformChainingAndPrintResult = false;
}  // namespace

Mapper::Mapper(const TransformInterpolationBuffer& odomToRangeSensorBuffer, std::shared_ptr<SubmapCollection> submaps)
    : odomToRangeSensorBuffer_(odomToRangeSensorBuffer), submaps_(submaps) {
  // `updates` with default parameters
  update(params_);
  double max_correspondence_distance = 0.4;
  small_registration_.reduction.num_threads = 8;
  small_registration_.rejector.max_dist_sq = max_correspondence_distance * max_correspondence_distance;
  small_registration_.criteria.rotation_eps = 0.005 * M_PI / 180.0;  // 0.001;
  small_registration_.criteria.translation_eps = 1e-3;               // 0.0008;
  small_registration_.optimizer.max_iterations = 30;
  small_registration_.optimizer.verbose = false;
}

void Mapper::setParameters(const MapperParameters& p) {
  params_ = p;
  update(p);
}

MapperParameters* Mapper::getParametersPtr() {
  return &params_;
}

void Mapper::setExternalOdometryFrameToCloudFrameCalibration(const Eigen::Isometry3d& transform) {
  // Not thread safe?
  calibration_ = transform.matrix();
  isCalibrationSet_ = true;
  return;
}

bool Mapper::isExternalOdometryFrameToCloudFrameCalibrationSet() {
  return isCalibrationSet_;
}

void Mapper::loopClosureUpdate(const Transform& loopClosureCorrection) {
  mapToRangeSensor_ = loopClosureCorrection * mapToRangeSensor_;
  mapToRangeSensorPrev_ = loopClosureCorrection * mapToRangeSensorPrev_;
}

bool Mapper::hasProcessedMeasurements() const {
  return !mapToRangeSensorBuffer_.empty();
}

void Mapper::update(const MapperParameters& p) {
  scan2MapReg_ = scanToMapRegistrationFactory(p);
  submaps_->setParameters(p);
}

Transform Mapper::getMapToOdom(const Time& timestamp) const {
  const Transform odomToRangeSensor = getTransform(timestamp, odomToRangeSensorBuffer_);
  const Transform mapToRangeSensor = getTransform(timestamp, mapToRangeSensorBuffer_);
  return mapToRangeSensor * odomToRangeSensor.inverse();
}

Transform Mapper::getMapToRangeSensor(const Time& timestamp) const {
  return getTransform(timestamp, mapToRangeSensorBuffer_);
}

bool Mapper::isRegistrationBestGuessBufferEmpty() const {
  return bestGuessBuffer_.empty();
}

Transform Mapper::getRegistrationBestGuess(const Time& timestamp) const {
  return getTransform(timestamp, bestGuessBuffer_);
}

const SubmapCollection& Mapper::getSubmaps() const {
  return *submaps_;
}

const Submap& Mapper::getActiveSubmap() const {
  return submaps_->getActiveSubmap();
}

const TransformInterpolationBuffer& Mapper::getMapToRangeSensorBuffer() const {
  return mapToRangeSensorBuffer_;
}

SubmapCollection* Mapper::getSubmapsPtr() {
  return submaps_.get();
}

void Mapper::setMapToRangeSensor(const Transform& t) {
  mapToRangeSensor_ = t;
}
void Mapper::setMapToRangeSensorInitial(const Transform& t) {
  if (isNewValueSetMapper_) {
    return;
  }

  std::cout << " Setting initial value transform at mapper: "
            << "\033[92m" << asString(t) << " \n"
            << "\033[0m";

  mapToRangeSensorPrev_ = t;
  mapToRangeSensor_ = t;
  isNewValueSetMapper_ = true;
  isInitialTransformSet_ = true;
}

const PointCloud& Mapper::getPreprocessedScan() const {
  return preProcessedScan_;
}

const ScanToMapRegistration& Mapper::getScanToMapRegistration() const {
  return *scan2MapReg_;
}

bool Mapper::addRangeMeasurement(const Mapper::PointCloud& rawScan, const Time& timestamp) {
  ProfilerScopeGuard main_scope("Mapper::addRangeMeasurement", "/tmp/slam_profile.csv",
                                "timestamp=" + std::to_string(toUniversal(timestamp)));

  if (!params_.isUseInitialMap_) {
    ProfilerScopeGuard scope("checkCalibration", "/tmp/slam_profile.csv");
    if (!isCalibrationSet_) {
      std::cerr << "Calibration is not set. Returning from mapping." << std::endl;
      return false;
    }
  }

  {
    ProfilerScopeGuard scope("setMapToRangeSensor", "/tmp/slam_profile.csv");
    submaps_->setMapToRangeSensor(mapToRangeSensor_);
  }

  if (submaps_->getNumSubmaps() == 0) {
    ProfilerScopeGuard scope("insertInitialScan", "/tmp/slam_profile.csv");

    if (params_.isUseInitialMap_) {
      assert_true(scan2MapReg_->isMergeScanValid(rawScan), "Init map invalid!!!!");
      submaps_->insertScan(rawScan, rawScan, mapToRangeSensor_, timestamp);
    } else {
      mapToRangeSensorPrev_ = mapToRangeSensor_;
      ProcessedScans processed = scan2MapReg_->processForScanMatchingAndMerging(rawScan, mapToRangeSensor_, true);

      {
        ProfilerScopeGuard scope_source_init("sourceInit", "/tmp/slam_profile.csv",
                                             "size=" + std::to_string(processed.merge_->points_.size()));

        source_ = std::make_shared<small_gicp::PointCloud>(processed.merge_->points_);
        // c_s = computeCentroid(*source_);
        // translatePointCloud(*source_, -c_s);
      }

      {
        ProfilerScopeGuard scope_source_kdtree("sourceKdTree", "/tmp/slam_profile.csv");
        source_tree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(source_, small_gicp::KdTreeBuilderOMP(4));
      }

      {
        ProfilerScopeGuard scope_source_normals("sourceNormals", "/tmp/slam_profile.csv");

        small_gicp::PointCloud& dst = *source_;  // always small‑gicp
        const auto& src_ptr = processed.merge_;  // OPEN3D ptr
        // const std::size_t N = dst.size();

        const int num_threads = (small_registration_.reduction.num_threads > 0) ? small_registration_.reduction.num_threads
                                                                                : static_cast<int>(std::thread::hardware_concurrency());

        estimate_normals_omp(dst, *source_tree_, /*knn=*/10, /*threads=*/num_threads);

        const size_t N = dst.normals.size();
        if (processed.merge_->normals_.size() != N) processed.merge_->normals_.resize(N);

        constexpr size_t OMP_THRESHOLD = 8192;
        if (N >= OMP_THRESHOLD) {
#pragma omp parallel for schedule(static)
          for (std::int64_t i = 0; i < static_cast<std::int64_t>(N); ++i) {
            // Efficient: Copy first 3 doubles from dst.normals[i] to processed.match_->normals_[i]
            std::memcpy(processed.merge_->normals_[i].data(), dst.normals[i].data(), 3 * sizeof(double));
          }
        } else {
          for (size_t i = 0; i < N; ++i) {
            std::memcpy(processed.merge_->normals_[i].data(), dst.normals[i].data(), 3 * sizeof(double));
          }
        }
      }

      submaps_->insertScan(rawScan, *processed.merge_, mapToRangeSensor_, timestamp);
      mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
      bestGuessBuffer_.push(timestamp, mapToRangeSensor_);
    }
    return true;
  }

  if (timestamp <= lastMeasurementTimestamp_) {
    ProfilerScopeGuard scope("handleOutOfOrder", "/tmp/slam_profile.csv");
    std::cerr << "\n\n !!!!! MAPPER WARNING: Measurements came out of order!!!! \n\n";

    int64_t uts_timestamp = toUniversal(timestamp);
    int64_t ns_since_unix_epoch = (uts_timestamp - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
    std::cout << " Current Timestamp: \033[92m" << ns_since_unix_epoch << " \n\033[0m";

    int64_t uts_timestamp_prev = toUniversal(lastMeasurementTimestamp_);
    int64_t ns_since_unix_epoch_prev = (uts_timestamp_prev - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
    std::cout << " Last Measurement Timestamp: \033[92m" << ns_since_unix_epoch_prev << " \n\033[0m";

    int64_t diff_millisecond_since_unix_epoch = (ns_since_unix_epoch - ns_since_unix_epoch_prev) / 1000000ll;
    std::cout << " Difference in Millisecond: \033[92m" << diff_millisecond_since_unix_epoch << " \n\033[0m";

    Time arbitraryLatestTime = odomToRangeSensorBuffer_.latest_time();
    Transform odomToRangeSensor = getTransform(arbitraryLatestTime, odomToRangeSensorBuffer_) * calibration_.inverse();
    Transform odomToRangeSensorPrev = getTransform(lastMeasurementTimestamp_, odomToRangeSensorBuffer_) * calibration_.inverse();
    Transform odometryMotion = odomToRangeSensorPrev.inverse() * odomToRangeSensor;
    Transform backupTransform = mapToRangeSensorPrev_ * odometryMotion;

    mapToRangeSensor_.matrix() = backupTransform.matrix();
    mapToRangeSensorBuffer_.push(arbitraryLatestTime, mapToRangeSensor_);
    bestGuessBuffer_.push(arbitraryLatestTime, mapToRangeSensorPrev_);
    submaps_->setMapToRangeSensor(mapToRangeSensor_);
    mapToRangeSensorPrev_ = mapToRangeSensor_;
    return true;
  }

  {
    ProfilerScopeGuard scope("checkTransformBuffer", "/tmp/slam_profile.csv");
    bool isOdomOkay = odomToRangeSensorBuffer_.has_query(timestamp);
    if (!isOdomOkay) {
      std::cerr << "WARNING: odomToRangeSensorBuffer_ DOES NOT HAVE THE DESIRED TRANSFORM! \n";
      std::cerr << "Going to attempt the scan to map refinement anyway. \n";
    }
    checkTransformChainingAndPrintResult(isCheckTransformChainingAndPrintResult);
  }

  Transform mapToRangeSensorEstimate = mapToRangeSensorPrev_;

  {
    ProfilerScopeGuard scope("odometryEstimation", "/tmp/slam_profile.csv");
    if (!isNewValueSetMapper_ && !isIgnoreOdometryPrediction_ && odomToRangeSensorBuffer_.has(timestamp)) {
      Transform odomToRangeSensor = getTransform(timestamp, odomToRangeSensorBuffer_) * calibration_.inverse();
      Transform odomToRangeSensorPrev = getTransform(lastMeasurementTimestamp_, odomToRangeSensorBuffer_) * calibration_.inverse();
      Transform odometryMotion = odomToRangeSensorPrev.inverse() * odomToRangeSensor;
      odometryMotionMemory_ = odometryMotion;
      mapToRangeSensorEstimate = mapToRangeSensorPrev_ * odometryMotion;
    }
  }

  isIgnoreOdometryPrediction_ = false;
  ProcessedScans processed;
  {
    ProfilerScopeGuard scope("scanPreprocessing", "/tmp/slam_profile.csv");
    processed = scan2MapReg_->processForScanMatchingAndMerging(rawScan, mapToRangeSensor_, false);
  }

  Transform correctedTransform_o3d;
  {
    // Compute time since last reference update
    double passedTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(timestamp - lastReferenceInitializationTimestamp_).count() / 1e3;

    // Compute relative motion since last reference update
    Transform relative = lastReferenceInitializationPose_.inverse() * mapToRangeSensor_;
    double translation = relative.translation().norm();
    double rotation = Eigen::AngleAxisd(relative.linear()).angle();

    // Configurable thresholds
    const double minTrans = 0.5;  // meters
    // Convert degrees to radians for minRot parameter
    const double minRot = 5.0 * M_PI / 180.0;                                           // degrees to radians
    const double minInterval = params_.scanMatcher_.icp_.referenceCloudSettingPeriod_;  // seconds

    bool enoughMotion = (translation >= minTrans) || (rotation >= minRot);
    bool enoughTime = (passedTime >= minInterval);
    bool updateReference = isNewValueSetMapper_ || (enoughMotion && enoughTime);

    if (updateReference) {
      PointCloudPtr mapPatch;
      {
        ProfilerScopeGuard scope_crop("cropSubmap", "/tmp/slam_profile.csv");

        if (submaps_->getNumSubmaps() == 1) {
          // Only one submap, crop from the active submap
          mapPatch = scan2MapReg_->cropSubmap(submaps_->getActiveSubmap(), mapToRangeSensor_, false);
        } else {
          size_t active = submaps_->activeSubmapIdx_;

          /* K-1 = 1 neighbour  →  total K = 2 maps in the composite */
          std::vector<size_t> nbrs = submaps_->findKClosestSubmaps(mapToRangeSensor_,
                                                                   /*k=*/2,  // number of neighbours you want
                                                                   active);  // <-- exclude only for search

          nbrs.push_back(active);
          mapPatch = submaps_->getCachedCompositeSubmapFromMulti(nbrs);

          // TODO needs work.
          // mapPatch = submaps_->getCachedCompositeSubmapFromVoxel(nbrs);
        }
      }

      if (mapPatch->IsEmpty()) {
        std::cerr << "\033[1;31m[Mapper] mapPatch is empty! Skipping scan matching.\033[0m" << std::endl;
        return false;
      }

      std::lock_guard<std::mutex> lck(mapManipulationMutex_);
      lastReferenceInitializationTimestamp_ = timestamp;
      lastReferenceInitializationPose_ = mapToRangeSensor_;  // save pose for next time

      {
        ProfilerScopeGuard scope_target_init("targetInit", "/tmp/slam_profile.csv", "size=" + std::to_string(mapPatch->points_.size()));
        target_ = std::make_shared<small_gicp::PointCloud>(mapPatch->points_);
      }

      {
        ProfilerScopeGuard scope_target_kdtree("targetKdTree", "/tmp/slam_profile.csv");
        target_tree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(target_, small_gicp::KdTreeBuilderOMP(8));
      }

      {
        ProfilerScopeGuard scope_target_cov("targetNormals", "/tmp/slam_profile.csv");
        copyOrEstimateNormals(*mapPatch, *target_, *target_tree_, /*knn=*/10, /*num_threads=*/8);
      }
    }

    {
      ProfilerScopeGuard scope_source_init("sourceInit", "/tmp/slam_profile.csv",
                                           "size=" + std::to_string(processed.merge_->points_.size()));

      source_ = std::make_shared<small_gicp::PointCloud>(processed.merge_->points_);
      // c_s = computeCentroid(*source_);
      // translatePointCloud(*source_, -c_s);
    }

    {
      ProfilerScopeGuard scope_source_normals("sourceNormals", "/tmp/slam_profile.csv");

      small_gicp::PointCloud& dst = *source_;  // always small‑gicp
      const auto& src_ptr = processed.merge_;  // OPEN3D ptr
      const std::size_t N = dst.size();

      /* Are normals available on the *Open3D* cloud? ------------------- */
      if (src_ptr->HasNormals() && src_ptr->normals_.size() == N) {
        if (dst.normals.size() != N) dst.normals.resize(N);

        assert((reinterpret_cast<uintptr_t>(dst.normals.data()) & 0xF) == 0 && "dst.normals is not 16‑byte aligned");

        constexpr std::size_t OMP_THRESHOLD = 8192;

        if (N >= OMP_THRESHOLD) {
#pragma omp parallel for schedule(static)
          for (std::int64_t i = 0; i < static_cast<std::int64_t>(N); ++i) {
            /* Open3D keeps normals in Eigen::Vector3d, so copy three doubles */
            std::memcpy(dst.normals[i].data(), src_ptr->normals_[i].data(), 3 * sizeof(double));
            dst.normals[i][3] = 0.0;
          }
        } else {
          for (std::size_t i = 0; i < N; ++i) {
            std::memcpy(dst.normals[i].data(), src_ptr->normals_[i].data(), 3 * sizeof(double));
            dst.normals[i][3] = 0.0;
          }
        }
      }
      /* --------------------------------------------------------------- */
      else {
        ProfilerScopeGuard scope_source_normals("sourceNormalsCalculatedAndCopied", "/tmp/slam_profile.csv");
        const int num_threads = (small_registration_.reduction.num_threads > 0) ? small_registration_.reduction.num_threads
                                                                                : static_cast<int>(std::thread::hardware_concurrency());

        {
          // Source tree is only needed for the normals calculation
          ProfilerScopeGuard scope_source_kdtree("sourceKdTree", "/tmp/slam_profile.csv");
          source_tree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(source_, small_gicp::KdTreeBuilderOMP(4));
        }

        estimate_normals_omp(dst, *source_tree_, /*knn=*/10, /*threads=*/num_threads);

        const size_t N = dst.normals.size();
        if (processed.merge_->normals_.size() != N) processed.merge_->normals_.resize(N);

        constexpr size_t OMP_THRESHOLD = 8192;
        if (N >= OMP_THRESHOLD) {
#pragma omp parallel for schedule(static)
          for (std::int64_t i = 0; i < static_cast<std::int64_t>(N); ++i) {
            // Efficient: Copy first 3 doubles from dst.normals[i] to processed.match_->normals_[i]
            std::memcpy(processed.merge_->normals_[i].data(), dst.normals[i].data(), 3 * sizeof(double));
          }
        } else {
          for (size_t i = 0; i < N; ++i) {
            std::memcpy(processed.merge_->normals_[i].data(), dst.normals[i].data(), 3 * sizeof(double));
          }
        }
      }
    }

    {
      ProfilerScopeGuard scope_icp_align("icpAlign", "/tmp/slam_profile.csv");
      std::lock_guard<std::mutex> lck(mapManipulationMutex_);

      // Eigen::Isometry3d init_T_target_source =
      //     Eigen::Translation3d(-c_t) * Eigen::Isometry3d(mapToRangeSensorEstimate.matrix()) * Eigen::Translation3d(c_s);

      Eigen::Isometry3d init_T_target_source(mapToRangeSensorEstimate.matrix());
      auto result = small_registration_.align(*target_, *source_, *target_tree_, init_T_target_source);
      // result.T_target_source = Eigen::Translation3d(c_t) * result.T_target_source * Eigen::Translation3d(-c_s);
      correctedTransform_o3d.matrix() = result.T_target_source.matrix();

      // Check if the result transform differs too much from the initial transform
      double translation_diff = (result.T_target_source.translation() - init_T_target_source.translation()).norm();
      double rotation_diff = Eigen::AngleAxisd(result.T_target_source.linear().transpose() * init_T_target_source.linear()).angle();

      const double max_translation_diff = 2.0;               // meters
      const double max_rotation_diff = 50.0 * M_PI / 180.0;  // 30 degrees in radians

      if (translation_diff > max_translation_diff || rotation_diff > max_rotation_diff) {
        std::cerr << "\n\n\033[1;41;97m"
                  << "!!!!!!! ICP result transform differs too much from initial guess: "
                  << "translation diff = " << translation_diff << ", rotation diff (deg) = " << (rotation_diff * 180.0 / M_PI)
                  << ". Using initial guess instead! "
                  << "\033[0m\n\n";
        correctedTransform_o3d.matrix() = init_T_target_source.matrix();
      }
    }
  }

  if (isNewValueSetMapper_) {
    if (isTimeValid(timestamp)) initTime_ = timestamp;

    mapToRangeSensorPrev_ = mapToRangeSensor_;
    mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
    bestGuessBuffer_.push(timestamp, mapToRangeSensorEstimate);
    isNewValueSetMapper_ = false;
    isIgnoreOdometryPrediction_ = true;
    return true;
  }

  preProcessedScan_ = *processed.merge_;
  // preProcessedScan_ = *processed.match_;
  mapToRangeSensor_.matrix() = correctedTransform_o3d.matrix();
  mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
  bestGuessBuffer_.push(timestamp, mapToRangeSensorEstimate);
  submaps_->setMapToRangeSensor(mapToRangeSensor_);

  {
    ProfilerScopeGuard scope("scanInsertionCheck", "/tmp/slam_profile.csv");
    double timeSinceInit = toSecondsSinceFirstMeasurement(timestamp) - toSecondsSinceFirstMeasurement(initTime_);
    if ((params_.isUseInitialMap_ && !params_.isMergeScansIntoMap_) ||
        (timeSinceInit < params_.mapMergeDelayInSeconds_ && params_.isUseInitialMap_ && params_.isMergeScansIntoMap_)) {
      lastMeasurementTimestamp_ = timestamp;
      mapToRangeSensorPrev_ = mapToRangeSensor_;
      return true;
    }
  }

  {
    ProfilerScopeGuard scope("scanInsertion(mapper)", "/tmp/slam_profile.csv");
    // scanInsertionTimer_.startStopwatch();

    Transform sensorMotion = mapToRangeSensorLastScanInsertion_.inverse() * mapToRangeSensor_;
    if (sensorMotion.translation().norm() >= params_.minMovementBetweenMappingSteps_) {
      submaps_->insertScan(rawScan, *processed.merge_, mapToRangeSensor_, timestamp);
      mapToRangeSensorLastScanInsertion_ = mapToRangeSensor_;
    }

    lastMeasurementTimestamp_ = timestamp;
    mapToRangeSensorPrev_ = mapToRangeSensor_;

    // double insertiontimeElapsed = scanInsertionTimer_.elapsedMsecSinceStopwatchStart();
    // scanInsertionTimer_.addMeasurementMsec(insertiontimeElapsed);
  }

  return true;
}

void Mapper::copyOrEstimateNormals(const open3d::geometry::PointCloud& src, small_gicp::PointCloud& dst,
                                   small_gicp::KdTree<small_gicp::PointCloud>& dst_tree, int normal_knn, int num_threads) {
  const std::size_t N = dst.size();
  assert(N == src.points_.size());

  constexpr std::size_t OMP_THRESHOLD = 16384;  // Use single-threaded below this
  constexpr std::size_t BLOCK_SIZE = 256;       // Cache-aware block size for OMP scheduling

  if (src.HasNormals() && src.normals_.size() == N) {
    if (dst.normals.size() != N) dst.normals.resize(N);

    assert((reinterpret_cast<uintptr_t>(dst.normals.data()) & 0xF) == 0 && "dst.normals is not 16-byte aligned");

    const Eigen::Vector3d* src_ptr = src.normals_.data();
    Eigen::Vector4d* dst_ptr = dst.normals.data();

    if (N < OMP_THRESHOLD || (num_threads == 1)) {
      // Single-threaded copy for small clouds
      for (std::size_t i = 0; i < N; ++i) {
        std::memcpy(dst_ptr[i].data(), src_ptr[i].data(), 3 * sizeof(double));
        dst_ptr[i][3] = 0.0;
      }
    } else {
      if (num_threads <= 0) num_threads = static_cast<int>(std::thread::hardware_concurrency());
// Multi-threaded, cache-blocked copy
#pragma omp parallel for schedule(static, BLOCK_SIZE) num_threads(num_threads)
      for (std::int64_t i = 0; i < static_cast<std::int64_t>(N); ++i) {
        std::memcpy(dst_ptr[i].data(), src_ptr[i].data(), 3 * sizeof(double));
        dst_ptr[i][3] = 0.0;
      }
    }
  } else {
    // Fallback: estimate normals if Open3D cloud lacks them
    if (num_threads <= 0) num_threads = static_cast<int>(std::thread::hardware_concurrency());
    estimate_normals_omp(dst, dst_tree, normal_knn, num_threads);
  }
}

Eigen::Vector3d Mapper::computeCentroid(const small_gicp::PointCloud& pc) {
  const size_t N = pc.size();
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();

#pragma omp parallel for reduction(vec3d_plus : sum) schedule(static)
  for (std::int64_t i = 0; i < static_cast<std::int64_t>(N); ++i) {
    sum += pc.point(i).head<3>();
  }

  return sum / static_cast<double>(N);
}

void Mapper::translatePointCloud(small_gicp::PointCloud& pc, const Eigen::Vector3d& t) {
  const size_t N = pc.size();

#pragma omp parallel for simd schedule(static)
  for (std::int64_t i = 0; i < static_cast<std::int64_t>(N); ++i) {
    pc.point(i).head<3>() += t;
  }
}

Mapper::PointCloud Mapper::getAssembledMapPointCloudVisualization() const {
  const size_t nSubmaps = submaps_->getNumSubmaps();
  if (nSubmaps == 0) {
    std::cerr << "No submaps. Returning empty cloud." << std::endl;
    return PointCloud();
  }

  std::vector<size_t> per_submap_sizes(nSubmaps, 0);
  std::vector<std::vector<Eigen::Vector3d>> thread_points(nSubmaps);

#pragma omp parallel for
  for (int i = 0; i < static_cast<int>(nSubmaps); ++i) {
    const auto& submap = submaps_->getSubmap(i);
    const auto& voxel_map = submap.getVoxelMap();
    auto& points_out = thread_points[i];

    // Check if voxel_map has any occupied voxels
    size_t voxel_count = 0;
    for (const auto& kv : voxel_map.voxels_) {
      if (!kv.second.idxs_.empty()) ++voxel_count;
    }

    if (voxel_count > 0) {
      // Use voxel map centers
      points_out.reserve(voxel_count);
      for (const auto& kv : voxel_map.voxels_) {
        if (!kv.second.idxs_.empty()) {
          points_out.push_back(voxel_map.getCenterFromKey(kv.first));
        }
      }
    } else {
      // Fallback: use raw point cloud
      const auto& pc = submap.getMapPointCloud();
      points_out.assign(pc.points_.begin(), pc.points_.end());
    }
    per_submap_sizes[i] = points_out.size();
  }

  // Print verbose info
  size_t total_points = 0;
  for (size_t i = 0; i < nSubmaps; ++i) {
    // std::cout << "[AssembledMap] Submap " << i << " has " << per_submap_sizes[i] << " points." << std::endl;
    total_points += per_submap_sizes[i];
  }
  // std::cout << "[AssembledMap] Total points in assembled map: " << total_points << std::endl;

  // Merge results
  PointCloud cloud;
  cloud.points_.reserve(total_points);
  for (auto& pts : thread_points) {
    cloud.points_.insert(cloud.points_.end(), std::make_move_iterator(pts.begin()), std::make_move_iterator(pts.end()));
  }
  return cloud;
}

Mapper::PointCloud Mapper::getAssembledMapPointCloud() const {
  const int nPoints = submaps_->getTotalNumPoints();

  if (nPoints == 0) {
    std::cerr << "No points in the assembled map. Returning empty cloud." << std::endl;
    return PointCloud();
  }

  PointCloud cloud;
  const Submap& activeSubmap = getActiveSubmap();
  cloud.points_.reserve(nPoints);
  if (activeSubmap.getMapPointCloud().HasColors()) {
    cloud.colors_.reserve(nPoints);
  }
  if (activeSubmap.getMapPointCloud().HasNormals()) {
    cloud.normals_.reserve(nPoints);
  }

  for (size_t j = 0; j < submaps_->getNumSubmaps(); ++j) {
    const PointCloud submap = submaps_->getSubmap(j).getMapPointCloudCopy();
    for (size_t i = 0; i < submap.points_.size(); ++i) {
      cloud.points_.push_back(submap.points_.at(i));
      if (submap.HasColors()) {
        cloud.colors_.push_back(submap.colors_.at(i));
      }
      if (submap.HasNormals()) {
        cloud.normals_.push_back(submap.normals_.at(i));
      }
    }
  }

  return cloud;
}

void Mapper::checkTransformChainingAndPrintResult(bool isCheckTransformChainingAndPrintResult) const {
  if (isCheckTransformChainingAndPrintResult && odomToRangeSensorBuffer_.size() > 70 && mapToRangeSensorBuffer_.size() > 70) {
    /*const auto odom1 = odomToRangeSensorBuffer_.latest_measurement(60).transform_;
    const auto odom2 = odomToRangeSensorBuffer_.latest_measurement(20).transform_;
    const auto start = mapToRangeSensorBuffer_.latest_measurement(60).transform_;
    const auto gt = mapToRangeSensorBuffer_.latest_measurement(20).transform_;
    const Transform mapMotion = start.inverse() * gt;
    const Transform odomMotion = odom1.inverse() * odom2;
    std::cout << "start      :  " << asString(start) << "\n";
    std::cout << "gt         :  " << asString(gt) << "\n";
    std::cout << "gt computed:  " << asString(start * mapMotion) << "\n";
    std::cout << "est        : " << asString(start * odomMotion) << "\n\n";*/
  }
}

} /* namespace o3d_slam */
