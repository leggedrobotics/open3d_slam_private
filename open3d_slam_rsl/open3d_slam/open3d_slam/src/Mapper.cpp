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
namespace o3d_slam {

namespace {
namespace registration = open3d::pipelines::registration;
const bool isCheckTransformChainingAndPrintResult = false;
}  // namespace

Mapper::Mapper(const TransformInterpolationBuffer& odomToRangeSensorBuffer, std::shared_ptr<SubmapCollection> submaps)
    : odomToRangeSensorBuffer_(odomToRangeSensorBuffer), submaps_(submaps) {
  // `updates` with default parameters
  update(params_);

  small_registration_.reduction.num_threads = 8;
  small_registration_.rejector.max_dist_sq = 1.0;
  small_registration_.criteria.rotation_eps = 0.001;
  small_registration_.criteria.translation_eps = 0.001;
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

  if (submaps_->getActiveSubmap().isEmpty()) {
    ProfilerScopeGuard scope("insertInitialScan", "/tmp/slam_profile.csv");

    if (params_.isUseInitialMap_) {
      assert_true(scan2MapReg_->isMergeScanValid(rawScan), "Init map invalid!!!!");
      submaps_->insertScan(rawScan, rawScan, mapToRangeSensor_, timestamp);
    } else {
      mapToRangeSensorPrev_ = mapToRangeSensor_;
      ProcessedScans processed = scan2MapReg_->processForScanMatchingAndMerging(rawScan, mapToRangeSensor_);
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
    bool isOdomOkay = odomToRangeSensorBuffer_.has(timestamp);
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
    // auxilaryTimer_.startStopwatch();
    processed = scan2MapReg_->processForScanMatchingAndMerging(rawScan, mapToRangeSensor_);
  }
  // double auxilarytimeElapsed = auxilaryTimer_.elapsedMsecSinceStopwatchStart();
  // auxilaryTimer_.addMeasurementMsec(auxilarytimeElapsed);

  Transform correctedTransform_o3d;
  {
    double passedTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(timestamp - lastReferenceInitializationTimestamp_).count() / 1e3;

    if (isNewValueSetMapper_ || (passedTime >= params_.scanMatcher_.icp_.referenceCloudSettingPeriod_)) {
      PointCloudPtr mapPatch;
      {
        ProfilerScopeGuard scope_crop("cropSubmap", "/tmp/slam_profile.csv");
        mapPatch = scan2MapReg_->cropSubmap(submaps_->getActiveSubmap(), mapToRangeSensor_);
      }

      if (mapPatch->IsEmpty()) return false;

      std::lock_guard<std::mutex> lck(mapManipulationMutex_);
      lastReferenceInitializationTimestamp_ = timestamp;

      {
        ProfilerScopeGuard scope_target_init("targetInit", "/tmp/slam_profile.csv");
        target_ = std::make_shared<small_gicp::PointCloud>(mapPatch->points_);
      }

      {
        ProfilerScopeGuard scope_target_kdtree("targetKdTree", "/tmp/slam_profile.csv");
        target_tree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(target_, small_gicp::KdTreeBuilderOMP(8));
      }

      {
        ProfilerScopeGuard scope_target_cov("targetCovariance", "/tmp/slam_profile.csv");
        estimate_covariances_omp(*target_, *target_tree_, 5, 8);
      }
    }

    {
      ProfilerScopeGuard scope_source_init("sourceInit", "/tmp/slam_profile.csv");
      source_ = std::make_shared<small_gicp::PointCloud>(processed.match_->points_);
    }

    {
      ProfilerScopeGuard scope_source_kdtree("sourceKdTree", "/tmp/slam_profile.csv");
      source_tree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(source_, small_gicp::KdTreeBuilderOMP(8));
    }

    {
      ProfilerScopeGuard scope_source_cov("sourceCovariance", "/tmp/slam_profile.csv");
      estimate_covariances_omp(*source_, *source_tree_, 10, 8);
    }

    {
      ProfilerScopeGuard scope_icp_align("icpAlign", "/tmp/slam_profile.csv");
      std::lock_guard<std::mutex> lck(mapManipulationMutex_);
      Eigen::Isometry3d init_T_target_source(mapToRangeSensorEstimate.matrix());
      auto result = small_registration_.align(*target_, *source_, *target_tree_, init_T_target_source);
      correctedTransform_o3d.matrix() = result.T_target_source.matrix();
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

  preProcessedScan_ = *processed.match_;
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
    ProfilerScopeGuard scope("scanInsertion", "/tmp/slam_profile.csv");
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

// Entry point to scan2map registration
// bool Mapper::addRangeMeasurement(const Mapper::PointCloud& rawScan, const Time& timestamp) {
//   if (!params_.isUseInitialMap_) {
//     if (!isCalibrationSet_) {
//       std::cerr << "Calibration is not set. Returning from mapping." << std::endl;
//       return false;
//     }
//   }

//   submaps_->setMapToRangeSensor(mapToRangeSensor_);

//   // insert first scan
//   if (submaps_->getActiveSubmap().isEmpty()) {
//     if (params_.isUseInitialMap_) {
//       assert_true(scan2MapReg_->isMergeScanValid(rawScan), "Init map invalid!!!!");
//       submaps_->insertScan(rawScan, rawScan, mapToRangeSensor_, timestamp);

//     } else {
//       mapToRangeSensorPrev_ = mapToRangeSensor_;
//       const ProcessedScans processed = scan2MapReg_->processForScanMatchingAndMerging(rawScan, mapToRangeSensor_);
//       // const ProcessedScans processed = scan2MapReg_->reducedProcessForScanMatchingAndMerging(rawScan, mapToRangeSensor_);

//       // TODO(TT) the init of submap is changed from identity to mapToRangeSensor_. This allows nice start of the mapping.
//       // Depending more on the initial transform.
//       submaps_->insertScan(rawScan, *processed.merge_, mapToRangeSensor_, timestamp);
//       mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
//       bestGuessBuffer_.push(timestamp, mapToRangeSensor_);
//     }
//     return true;
//   }

//   if (timestamp <= lastMeasurementTimestamp_) {
//     std::cerr << "\n\n !!!!! MAPPER WARNING: Measurements came out of order!!!! \n\n";
//     std::cerr << "Using the previously calculated odometry motion to propagate measurement. \n";

//     int64_t uts_timestamp = toUniversal(timestamp);
//     int64_t ns_since_unix_epoch = (uts_timestamp - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
//     std::cout << " Current Timestamp: "
//               << "\033[92m" << ns_since_unix_epoch << " \n"
//               << "\033[0m";

//     int64_t uts_timestamp_prev = toUniversal(lastMeasurementTimestamp_);
//     int64_t ns_since_unix_epoch_prev = (uts_timestamp_prev - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
//     std::cout << " Last Measurement Timestamp: "
//               << "\033[92m" << ns_since_unix_epoch_prev << " \n"
//               << "\033[0m";

//     int64_t diff_millisecond_since_unix_epoch = (ns_since_unix_epoch - ns_since_unix_epoch_prev) / 1000000ll;

//     std::cout << " Difference in Millisecond: "
//               << "\033[92m" << diff_millisecond_since_unix_epoch << " \n"
//               << "\033[0m";

//     // Latest arbitrary time:
//     Time arbitraryLatestTime = odomToRangeSensorBuffer_.latest_time();

//     const Transform odomToRangeSensor = getTransform(arbitraryLatestTime, odomToRangeSensorBuffer_) * calibration_.inverse();
//     const Transform odomToRangeSensorPrev = getTransform(lastMeasurementTimestamp_, odomToRangeSensorBuffer_) * calibration_.inverse();
//     Transform odometryMotion = odomToRangeSensorPrev.inverse() * odomToRangeSensor;
//     Transform backupTransform = mapToRangeSensorPrev_ * odometryMotion;

//     // Pass to placeholders variables
//     mapToRangeSensor_.matrix() = backupTransform.matrix();
//     mapToRangeSensorBuffer_.push(arbitraryLatestTime, mapToRangeSensor_);
//     bestGuessBuffer_.push(arbitraryLatestTime, mapToRangeSensorPrev_);
//     submaps_->setMapToRangeSensor(mapToRangeSensor_);
//     // lastMeasurementTimestamp_ = arbitraryLatestTime;
//     mapToRangeSensorPrev_ = mapToRangeSensor_;

//     return true;
//   }

//   bool isOdomOkay = odomToRangeSensorBuffer_.has(timestamp);
//   if (!isOdomOkay) {
//     std::cerr << "WARNING: odomToRangeSensorBuffer_ DOES NOT HAVE THE DESIRED TRANSFORM! \n";
//     std::cerr << "Going to attempt the scan to map refinement anyway. \n";
//   }

//   checkTransformChainingAndPrintResult(isCheckTransformChainingAndPrintResult);

//   Transform mapToRangeSensorEstimate = mapToRangeSensorPrev_;

//   // This is where we prepare the scan2map initial guess.
//   if (isOdomOkay && !isNewValueSetMapper_ && !isIgnoreOdometryPrediction_) {
//     // Get the pose at the current timestamp of the PC we are operating with.
//     const Transform odomToRangeSensor = getTransform(timestamp, odomToRangeSensorBuffer_) * calibration_.inverse();

//     // Get the pose at the previous PC timestamp we already calculated the scan2map refinement for.
//     const Transform odomToRangeSensorPrev = getTransform(lastMeasurementTimestamp_, odomToRangeSensorBuffer_) * calibration_.inverse();

//     // Calculate the motion between the two poses. This immediately allows us get rid of `drift` in the odometry.
//     Transform odometryMotion = odomToRangeSensorPrev.inverse() * odomToRangeSensor;
//     odometryMotionMemory_ = odometryMotion;

//     // Apply the calculated odometry motion to the previous scan2map refined pose.
//     mapToRangeSensorEstimate = mapToRangeSensorPrev_ * odometryMotion;

//     if (odometryMotion.translation().norm() == 0.0) {
//       std::cout << " Odometry MOTION SHOULDNT BE PERFECTLY 0. "
//                 << "\033[92m" << asString(odometryMotion) << " \n"
//                 << "\033[0m";
//     }

//     /*int64_t uts_timestamp = toUniversal(timestamp);
//     int64_t ns_since_unix_epoch = (uts_timestamp - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
//     std::cout << " timestamp: " << "\033[92m" << ns_since_unix_epoch << " \n" << "\033[0m";

//     int64_t uts_timestamp_prev = toUniversal(lastMeasurementTimestamp_);
//     int64_t ns_since_unix_epoch_prev = (uts_timestamp_prev - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
//     std::cout << " lastMeasurementTimestamp_: " << "\033[92m" << ns_since_unix_epoch_prev << " \n" << "\033[0m";

//     std::cout << " odomToRangeSensor: " << "\033[92m" << asString(odomToRangeSensor) << " \n" << "\033[0m";
//     std::cout << " odomToRangeSensorPrev: " << "\033[92m" << asString(odomToRangeSensorPrev) << " \n" << "\033[0m";
//     std::cout << " odometryMotion: " << "\033[92m" << asString(odometryMotion) << " \n" << "\033[0m";
//     */
//   }

//   isIgnoreOdometryPrediction_ = false;

//   // Start auxilary time measurement.
//   auxilaryTimer_.startStopwatch();

//   const ProcessedScans processed = scan2MapReg_->processForScanMatchingAndMerging(rawScan, mapToRangeSensor_);

//   const double auxilarytimeElapsed = auxilaryTimer_.elapsedMsecSinceStopwatchStart();
//   auxilaryTimer_.addMeasurementMsec(auxilarytimeElapsed);

//   if (params_.isPrintTimingStatistics_) {
//     std::cout << " Auxilary time: "
//               << "\033[92m" << auxilarytimeElapsed << " msec \n"
//               << "\033[0m";
//   }

//   // mapToRangeSensorEstimate.rotation().normalized();

//   // std::cout << "Number of points in submap: " << submaps_->getActiveSubmap().getNbPoints() << std::endl;

//   // Initialize the registered pose.
//   Transform correctedTransform_o3d;
//   try {
//     // std::cout << "activeSubmap: " << activeSubmap->dataPoints_.features.cols() << " x " << activeSubmap->dataPoints_.features.rows()
//     <<
//     // std::endl;

//     double passedTime =
//         std::chrono::duration_cast<std::chrono::milliseconds>(timestamp - lastReferenceInitializationTimestamp_).count() / 1e3;
//     // std::cerr << "Passed Time: " << passedTime << std::endl;

//     if (isNewValueSetMapper_ || (passedTime >= params_.scanMatcher_.icp_.referenceCloudSettingPeriod_)) {
//       {
//         // Crop the submap around the robot.
//         const PointCloudPtr mapPatch = scan2MapReg_->cropSubmap(submaps_->getActiveSubmap(), mapToRangeSensor_);

//         if (mapPatch->IsEmpty()) {
//           std::cout << "\033[92m"
//                     << " Map patch is empty impossible. Returning"
//                     << " \n"
//                     << "\033[0m";
//           return false;
//         }

//         std::lock_guard<std::mutex> lck(mapManipulationMutex_);

//         referenceInitTimer_.startStopwatch();
//         lastReferenceInitializationTimestamp_ = timestamp;

//         target_ = std::make_shared<small_gicp::PointCloud>(mapPatch->points_);
//         target_tree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
//             target_, small_gicp::KdTreeBuilderOMP(small_registration_.reduction.num_threads));
//         estimate_covariances_omp(*target_, *target_tree_, 10, small_registration_.reduction.num_threads);
//       }

//       const double referenceInittimeElapsed = referenceInitTimer_.elapsedMsecSinceStopwatchStart();
//       referenceInitTimer_.addMeasurementMsec(referenceInittimeElapsed);

//       if (params_.isPrintTimingStatistics_) {
//         std::cout << " Reference Cloud Re-init time: "
//                   << "\033[92m" << referenceInittimeElapsed << " msec \n"
//                   << "\033[0m";
//       }

//     } else {
//       referenceInitTimer_.reset();
//     }

//     source_ = std::make_shared<small_gicp::PointCloud>(processed.match_->points_);

//     // Create KdTree
//     source_tree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
//         source_, small_gicp::KdTreeBuilderOMP(small_registration_.reduction.num_threads));

//     // Estimate point covariances
//     estimate_covariances_omp(*source_, *source_tree_, 10, small_registration_.reduction.num_threads);

//     testmapperOnlyTimer_.startStopwatch();
//     // Pass the calculated transform to o3d transform.

//     // small_gicp::RegistrationSetting setting;
//     // setting.num_threads = 32;
//     // setting.max_correspondence_distance = 1.0;

//     {
//       std::lock_guard<std::mutex> lck(mapManipulationMutex_);

//       Eigen::Isometry3d init_T_target_source(mapToRangeSensorEstimate.matrix());
//       auto result = small_registration_.align(*target_, *source_, *target_tree_, init_T_target_source);
//       // auto result = small_gicp::align(*target_, *source_, *target_tree_, init_T_target_source, setting);
//       correctedTransform_o3d.matrix() = result.T_target_source.matrix();
//     }

//     if (firstRefinement_) {
//       std::cout << "\033[92m"
//                 << " Open3d SLAM is running properly."
//                 << " \n "
//                 << "\033[0m";
//       firstRefinement_ = false;
//     }

//     const double timeElapsed = testmapperOnlyTimer_.elapsedMsecSinceStopwatchStart();
//     testmapperOnlyTimer_.addMeasurementMsec(timeElapsed);

//     if (params_.isPrintTimingStatistics_) {
//       std::cout << " Scan2Map Registration: "
//                 << "\033[92m" << timeElapsed << " msec \n "
//                 << "\033[0m";
//     }

//   } catch (const std::runtime_error& error) {
//     std::cout << "Experienced a runtime error while running libpointmatcher ICP: " << error.what() << std::endl;
//   }

//   // std::cout << "preeIcp: " << asString(mapToRangeSensorEstimate) << "\n";
//   // std::cout << "postIcp xicp: " << asString(correctedTransform_o3d) << "\n\n";

//   if (isNewValueSetMapper_) {
//     if (isTimeValid(timestamp)) {
//       initTime_ = timestamp;
//     }

//     std::cout << "\033[92m"
//               << "Setting initial value for the map to range sensor transform. ONLY expected at the start-up."
//               << "\033[0m"
//               << "\n";
//     mapToRangeSensorPrev_ = mapToRangeSensor_;
//     mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
//     bestGuessBuffer_.push(timestamp, mapToRangeSensorEstimate);
//     isNewValueSetMapper_ = false;
//     isIgnoreOdometryPrediction_ = true;
//     return true;
//   }

//   // Pass to placeholders variables
//   preProcessedScan_ = *processed.match_;
//   mapToRangeSensor_.matrix() = correctedTransform_o3d.matrix();
//   mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
//   bestGuessBuffer_.push(timestamp, mapToRangeSensorEstimate);
//   submaps_->setMapToRangeSensor(mapToRangeSensor_);

//   // Given a map, we don't want to add the mis-aligned first scans to the map. Hence giving the registration time to converge.
//   double timeSinceInit = toSecondsSinceFirstMeasurement(timestamp) - toSecondsSinceFirstMeasurement(initTime_);
//   if ((params_.isUseInitialMap_ && !params_.isMergeScansIntoMap_) ||
//       (timeSinceInit < params_.mapMergeDelayInSeconds_ && params_.isUseInitialMap_ && params_.isMergeScansIntoMap_)) {
//     // Early return before inserting the scans.
//     lastMeasurementTimestamp_ = timestamp;
//     mapToRangeSensorPrev_ = mapToRangeSensor_;

//     if (params_.isPrintTimingStatistics_) {
//       std::cout << "o3d_slam Mapper: "
//                 << "\033[92m Skipping Merging to Map. \n "
//                 << "\033[0m";
//     }

//     return true;
//   }

//   scanInsertionTimer_.startStopwatch();
//   // Concatenate registered cloud into map
//   const Transform sensorMotion = mapToRangeSensorLastScanInsertion_.inverse() * mapToRangeSensor_;
//   const bool isMovedTooLittle = sensorMotion.translation().norm() < params_.minMovementBetweenMappingSteps_;
//   if (!isMovedTooLittle) {
//     // Timer t("scan_insertion_and_bookeeping");
//     submaps_->insertScan(rawScan, *processed.merge_, mapToRangeSensor_, timestamp);
//     mapToRangeSensorLastScanInsertion_ = mapToRangeSensor_;
//   }

//   lastMeasurementTimestamp_ = timestamp;
//   mapToRangeSensorPrev_ = mapToRangeSensor_;

//   const double insertiontimeElapsed = scanInsertionTimer_.elapsedMsecSinceStopwatchStart();
//   scanInsertionTimer_.addMeasurementMsec(insertiontimeElapsed);

//   if (params_.isPrintTimingStatistics_) {
//     std::cout << "Scan Insertion: "
//               << "\033[92m" << insertiontimeElapsed << " msec \n "
//               << "\033[0m";
//   }

//   return true;
// }

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
