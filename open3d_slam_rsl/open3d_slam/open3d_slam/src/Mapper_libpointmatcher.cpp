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
  double max_correspondence_distance = 1.0;
  small_registration_.reduction.num_threads = 8;
  small_registration_.rejector.max_dist_sq = max_correspondence_distance * max_correspondence_distance;
  small_registration_.criteria.rotation_eps = 0.001;
  small_registration_.criteria.translation_eps = 0.0008;
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
  const ProcessedScans processed = scan2MapReg_->processForScanMatchingAndMerging(rawScan, mapToRangeSensor_);
  auto croppedCloud = open3d_conversions::createSimilarPointmatcherCloud(processed.match_->points_.size());
  open3d_conversions::open3dToPointmatcher(*processed.match_, *croppedCloud);

  const double auxilarytimeElapsed = auxilaryTimer_.elapsedMsecSinceStopwatchStart();
  auxilaryTimer_.addMeasurementMsec(auxilarytimeElapsed);

  if (params_.isPrintTimingStatistics_) {
    std::cout << " Auxilary time: "
              << "\033[92m" << auxilarytimeElapsed << " msec \n"
              << "\033[0m";
  }

  // Convert the initial guess to pointmatcher
  pointmatcher_ros::PmTfParameters transformReadingToReferenceInitialGuess;
  transformReadingToReferenceInitialGuess.matrix() = mapToRangeSensorEstimate.matrix().cast<float>();

  // Initialize the registered pose.
  pointmatcher_ros::PmTfParameters correctedTransform = transformReadingToReferenceInitialGuess;

  try {
    // Crop the submap around the robot.
    const PointCloudPtr mapPatch = scan2MapReg_->cropSubmap(submaps_->getActiveSubmap(), mapToRangeSensor_);

    if (mapPatch->IsEmpty()) {
      std::cout << "\033[92m"
                << " Map patch is empty impossible. Returning"
                << " \n"
                << "\033[0m";
      return false;
    }

    double passedTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(timestamp - lastReferenceInitializationTimestamp_).count() / 1e3;

    if (isNewValueSetMapper_ || (passedTime >= params_.scanMatcher_.icp_.referenceCloudSettingPeriod_)) {
      {
        std::lock_guard<std::mutex> lck(mapManipulationMutex_);
        // Create pointmatcher cloud
        open3d_conversions::PmStampedPointCloud activeSubmapPm_ =
            *open3d_conversions::createSimilarPointmatcherCloud(mapPatch->points_.size());

        // Convert to pointmatcher. This is time consuming.
        open3d_conversions::open3dToPointmatcher(*mapPatch, activeSubmapPm_);

        referenceInitTimer_.startStopwatch();

        lastReferenceInitializationTimestamp_ = timestamp;

        if (!icp_.initReference(activeSubmapPm_.dataPoints_)) {
          std::cout << "Failed to initialize reference cloud. Exitting. " << std::endl;
          return false;
        }
      }

      const double referenceInittimeElapsed = referenceInitTimer_.elapsedMsecSinceStopwatchStart();
      referenceInitTimer_.addMeasurementMsec(referenceInittimeElapsed);

      if (params_.isPrintTimingStatistics_) {
        std::cout << " Reference Cloud Re-init time: "
                  << "\033[92m" << referenceInittimeElapsed << " msec \n"
                  << "\033[0m";
      }

    } else {
      referenceInitTimer_.reset();
    }

    testmapperOnlyTimer_.startStopwatch();

    // The +1000 is to prevent early triggering of the condition. Since points might decrease due to carving.
    // if(activeSubmapPm_->dataPoints_.features.cols() > croppedCloud->dataPoints_.features.cols() + 1000){

    {
      std::lock_guard<std::mutex> lck(mapManipulationMutex_);

      // We are explicitly setting the reference cloud above. Hence the empty placeholder.
      open3d_conversions::PmStampedPointCloud emptyPlaceholder;
      correctedTransform =
          icp_.compute(croppedCloud->dataPoints_, emptyPlaceholder.dataPoints_, transformReadingToReferenceInitialGuess, false);
    }

    if (firstRefinement_) {
      std::cout << "\033[92m"
                << " Open3d SLAM is running properly."
                << " \n "
                << "\033[0m";
      firstRefinement_ = false;
    }

    const double timeElapsed = testmapperOnlyTimer_.elapsedMsecSinceStopwatchStart();
    testmapperOnlyTimer_.addMeasurementMsec(timeElapsed);

    if (params_.isPrintTimingStatistics_) {
      std::cout << " Scan2Map Registration: "
                << "\033[92m" << timeElapsed << " msec \n "
                << "\033[0m";
    }

  } catch (const std::runtime_error& error) {
    std::cout << "Experienced a runtime error while running libpointmatcher ICP: " << error.what() << std::endl;
  }

  // TODO(TT) Add a failsafe checker for health?
  /*if (!params_.isIgnoreMinRefinementFitness_ && result.fitness_ < params_.scanMatcher_.minRefinementFitness_) {
    std::cout << "Skipping the refinement step, fitness: " << result.fitness_ << std::endl;
    std::cout << "preeIcp: " << asString(mapToRangeSensorEstimate) << "\n";
    std::cout << "postIcp: " << asString(Transform(result.transformation_)) << "\n\n";
    return false;
  }
  */

  // Pass the calculated transform to o3d transform.
  Transform correctedTransform_o3d;
  correctedTransform_o3d.matrix() = correctedTransform.matrix().cast<double>();

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
