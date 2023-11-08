/*
 * SlamWrapperRosRos.cpp
 *
 *  Created on: Apr 19, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/SlamWrapperRos.hpp"

#include <open3d/Open3D.h>
#include <chrono>
#include <fstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "nav_msgs/Odometry.h"
#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam/Mapper.hpp"
#include "open3d_slam/Odometry.hpp"
#include "open3d_slam/OptimizationProblem.hpp"
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/constraint_builders.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam_lua_io/parameter_loaders.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"

#ifdef open3d_slam_ros_OPENMP_FOUND
#include <omp.h>
#endif

namespace o3d_slam {

namespace {
// Frames were used to included here.
}

SlamWrapperRos::SlamWrapperRos(ros::NodeHandlePtr nh) : BASE(), nh_(nh) {
  tfBroadcaster_.reset(new tf2_ros::TransformBroadcaster());

  tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
  tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);

  prevPublishedTimeScanToScan_ = fromUniversal(0);
  prevPublishedTimeScanToMap_ = fromUniversal(0);

  baseToLidarTransform_.transform.rotation.w = 1.0;
  baseToLidarTransform_.transform.rotation.z = 0.0;
  baseToLidarTransform_.transform.rotation.y = 0.0;
  baseToLidarTransform_.transform.rotation.x = 0.0;

  baseToLidarTransform_.transform.translation.z = 0.0;
  baseToLidarTransform_.transform.translation.y = 0.0;
  baseToLidarTransform_.transform.translation.x = 0.0;

}

bool SlamWrapperRos::readCalibrationIfNeeded(){

  if (!isStaticTransformAttempted_ && (frames_.rangeSensorFrame != frames_.assumed_external_odometry_tracked_frame)){
    try {
      // Lookup duration is long because the transform is static.
      tfBuffer_->canTransform(frames_.rangeSensorFrame, frames_.assumed_external_odometry_tracked_frame, ros::Time::now(), ros::Duration(5.0));
      auto tfTransformation = tfBuffer_->lookupTransform(frames_.rangeSensorFrame, frames_.assumed_external_odometry_tracked_frame, ros::Time::now(), ros::Duration(0.1));
      ROS_INFO_STREAM("\033[92m" << "Found the transform between " << frames_.rangeSensorFrame << " and " << frames_.assumed_external_odometry_tracked_frame << "\033[0m");
      ROS_INFO_STREAM("\033[92m" << "You dont believe me? Here it is:\n " << tfTransformation << "\033[0m");
      
      // Set the frame transformation between the external odometry frame and the range sensor frame.
      mapper_->setExternalOdometryFrameToCloudFrameCalibration(tf2::transformToEigen(tfTransformation));

      /*bool odometryFound = false;

      while (!odometryFound)
      {
        geometry_msgs::PoseStamped odomPose_transformed;
        const Time latestScanToScan = latestScanToScanRegistrationTimestamp_;
        const bool isAlreadyPublished = latestScanToScan == prevPublishedTimeScanToScanOdom_;
        if (!isAlreadyPublished && odometry_->hasProcessedMeasurements()) {
          const Transform T = odometry_->getOdomToRangeSensor(latestScanToScan);

          geometry_msgs::PoseStamped odomPose;

          odomPose.pose =o3d_slam::getPose(T.matrix());

          tf2::doTransform(odomPose, odomPose_transformed, tfTransformation);

          geometry_msgs::PoseStamped initialPose = odomPose_transformed;

          //initialPose.position=odomPose.position;
          initialPose.pose.orientation.w=1.0;
          initialPose.pose.orientation.z=0.0;
          initialPose.pose.orientation.y=0.0;
          initialPose.pose.orientation.x=0.0;
          odometry_->setInitialTransform(o3d_slam::getTransform(initialPose.pose).matrix());
          mapper_->setMapToRangeSensorInitial(Transform(o3d_slam::getTransform(initialPose.pose).matrix()));
          odometryFound = true;


        }else{
          ROS_ERROR_STREAM("\033[92m" << "odometry no processed measurements\n " << "\033[0m");
        }
      }*/

      isStaticTransformAttempted_ = true;
    
    } catch (const tf2::TransformException& exception) {
      ROS_WARN_STREAM_THROTTLE(0.2,"Caught exception while looking for the transform. " << exception.what());
      return false;
    }
    return true;

  }else{
    isStaticTransformAttempted_ = true; // hmm
    return false;
  }
}

SlamWrapperRos::~SlamWrapperRos() {
  if (tfWorker_.joinable()) {
    tfWorker_.join();
    std::cout << "Joined tf worker \n";
  }
  if (visualizationWorker_.joinable()) {
    visualizationWorker_.join();
    std::cout << "Joined visualization worker \n";
  }
  if (params_.odometry_.isPublishOdometryMsgs_ && odomPublisherWorker_.joinable()) {
    odomPublisherWorker_.join();
    std::cout << "Joined odom publisher worker \n";
  }
}

void SlamWrapperRos::startWorkers() {
  tfWorker_ = std::thread([this]() { tfWorker(); });
  visualizationWorker_ = std::thread([this]() { visualizationWorker(); });
  if (params_.odometry_.isPublishOdometryMsgs_) {
    odomPublisherWorker_ = std::thread([this]() { odomPublisherWorker(); });
  }

  BASE::startWorkers();

  // Read the static calibration between the odometry frame and the range sensor frame.
  // TODO(TT) this can be from a file or through tf. Make it available, people are lazy to get tf.
  if (!isStaticTransformAttempted_){
    if(!readCalibrationIfNeeded()){
      ROS_ERROR("Can't find static transform. Exiting.");
      return;
    }
  }
}

void SlamWrapperRos::odomPublisherWorker() {
  ros::Rate r(500.0);
  while (ros::ok()) {
    auto getTransformMsg = [this](const Transform& T, const Time& t) {
      ros::Time timestamp = toRos(t);
      geometry_msgs::TransformStamped transformMsg = o3d_slam::toRos(T.matrix(), timestamp, frames_.mapFrame, frames_.rangeSensorFrame);
      return transformMsg;
    };

    auto getOdomMsg = [](const geometry_msgs::TransformStamped& transformMsg) {
      nav_msgs::Odometry odomMsg;
      odomMsg.header = transformMsg.header;
      odomMsg.child_frame_id = transformMsg.child_frame_id;
      odomMsg.pose.pose.orientation = transformMsg.transform.rotation;
      odomMsg.pose.pose.position.x = transformMsg.transform.translation.x;
      odomMsg.pose.pose.position.y = transformMsg.transform.translation.y;
      odomMsg.pose.pose.position.z = transformMsg.transform.translation.z;
      return odomMsg;
    };

    const Time latestScanToScan = latestScanToScanRegistrationTimestamp_;
    const bool isAlreadyPublished = latestScanToScan == prevPublishedTimeScanToScanOdom_;
    if (!isAlreadyPublished && odometry_->hasProcessedMeasurements()) {
      const Transform T = odometry_->getOdomToRangeSensor(latestScanToScan);
      geometry_msgs::TransformStamped transformMsg = getTransformMsg(T, latestScanToScan);
      nav_msgs::Odometry odomMsg = getOdomMsg(transformMsg);
      publishIfSubscriberExists(transformMsg, scan2scanTransformPublisher_);
      publishIfSubscriberExists(odomMsg, scan2scanOdomPublisher_);
      prevPublishedTimeScanToScanOdom_ = latestScanToScan;
    }

    const Time latestScanToMap = latestScanToMapRefinementTimestamp_;
    const bool isScanToMapAlreadyPublished = latestScanToMap == prevPublishedTimeScanToMapOdom_;
    if (!isScanToMapAlreadyPublished && mapper_->hasProcessedMeasurements()) {
      const Transform T = mapper_->getMapToRangeSensor(latestScanToMap);
      geometry_msgs::TransformStamped transformMsg = getTransformMsg(T, latestScanToMap);
      nav_msgs::Odometry odomMsg = getOdomMsg(transformMsg);
      publishIfSubscriberExists(transformMsg, scan2mapTransformPublisher_);
      publishIfSubscriberExists(odomMsg, scan2mapOdomPublisher_);
      prevPublishedTimeScanToMapOdom_ = latestScanToMap;
    }

    ros::spinOnce();
    r.sleep();
  }
}

void SlamWrapperRos::offlineTfWorker() {
  const Time latestScanToScan = latestScanToScanRegistrationTimestamp_;
  const bool isAlreadyPublished = latestScanToScan == prevPublishedTimeScanToScan_;
  if ((!isAlreadyPublished ) && (odometry_->hasProcessedMeasurements())) {
    const Transform T = odometry_->getOdomToRangeSensor(latestScanToScan);
    ros::Time timestamp = toRos(latestScanToScan);
    //o3d_slam::publishTfTransform(T.matrix(), timestamp, o3d_slam::odomFrame, frames_.rangeSensorFrame, tfBroadcaster_.get());
    o3d_slam::publishTfTransform(T.matrix(), timestamp, frames_.mapFrame, "raw_odom_o3d", tfBroadcaster_.get());
    prevPublishedTimeScanToScan_ = latestScanToScan;
  }

  const Time latestScanToMap = latestScanToMapRefinementTimestamp_;
  const bool isScanToMapAlreadyPublished = latestScanToMap == prevPublishedTimeScanToMap_;
  if (!isScanToMapAlreadyPublished && mapper_->hasProcessedMeasurements()) {
    publishMapToOdomTf(latestScanToMap);
    prevPublishedTimeScanToMap_ = latestScanToMap;
  }
}

void SlamWrapperRos::tfWorker() {
  ros::WallRate r(75.0);
  while (ros::ok()) {
    const Time latestScanToScan = latestScanToScanRegistrationTimestamp_;
    const bool isAlreadyPublished = latestScanToScan == prevPublishedTimeScanToScan_;
    if ((!isAlreadyPublished ) && (odometry_->hasProcessedMeasurements())) {
      const Transform T = odometry_->getOdomToRangeSensor(latestScanToScan);
      ros::Time timestamp = toRos(latestScanToScan);
      o3d_slam::publishTfTransform(T.matrix(), timestamp, frames_.odomFrame, frames_.rangeSensorFrame, tfBroadcaster_.get());
      o3d_slam::publishTfTransform(T.matrix(), timestamp, frames_.mapFrame, "raw_odom_o3d", tfBroadcaster_.get());
      prevPublishedTimeScanToScan_ = latestScanToScan;
    }

    const Time latestScanToMap = latestScanToMapRefinementTimestamp_;
    const bool isScanToMapAlreadyPublished = latestScanToMap == prevPublishedTimeScanToMap_;
    if (!isScanToMapAlreadyPublished && mapper_->hasProcessedMeasurements()) {
      publishMapToOdomTf(latestScanToMap);
      prevPublishedTimeScanToMap_ = latestScanToMap;

      if (trackedPathPub_.getNumSubscribers() > 0u || trackedPathPub_.isLatched()) {
        mapper_->trackedPath_.header.stamp = o3d_slam::toRos(latestScanToMap);
        mapper_->trackedPath_.header.frame_id = frames_.mapFrame;
        trackedPathPub_.publish(mapper_->trackedPath_);
      }

      if (bestGuessPathPub_.getNumSubscribers() > 0u || bestGuessPathPub_.isLatched()) {
        mapper_->bestGuessPath_.header.stamp = o3d_slam::toRos(latestScanToMap);
        mapper_->bestGuessPath_.header.frame_id = frames_.mapFrame;
        bestGuessPathPub_.publish(mapper_->bestGuessPath_);
      }
    }

    ros::spinOnce();
    r.sleep();
  }
}

void SlamWrapperRos::offlineVisualizationWorker() {
  const Time scanToScanTimestamp = latestScanToScanRegistrationTimestamp_;
  ros::Time timestamp = toRos(scanToScanTimestamp);
  o3d_slam::publishSubmapCoordinateAxes(mapper_->getSubmaps(), frames_.mapFrame, timestamp, submapOriginsPub_);

}

void SlamWrapperRos::visualizationWorker() {
  ros::WallRate r(20.0);
  while (ros::ok()) {
    const Time scanToScanTimestamp = latestScanToScanRegistrationTimestamp_;
    if (odometryInputPub_.getNumSubscribers() > 0 && isTimeValid(scanToScanTimestamp)) {
      const PointCloud odomInput = odometry_->getPreProcessedCloud();
      o3d_slam::publishCloud(odomInput, frames_.rangeSensorFrame, toRos(scanToScanTimestamp), odometryInputPub_);
    }

    const Time scanToMapTimestamp = latestScanToMapRefinementTimestamp_;
    if (isTimeValid(scanToMapTimestamp)) {
      publishDenseMap(scanToMapTimestamp);
      publishMaps(scanToMapTimestamp);
    }

    ros::spinOnce();
    r.sleep();
  }
}

bool SlamWrapperRos::readLibpointmatcherConfig(const std::string& path) {

  std::ifstream fileStream(path.c_str());
  if (!fileStream.good()) {
    std::cout<< "Cannot load ICP configuration from '" << path << "'.";
    return false;
  }
  mapper_->icp_.loadFromYaml(fileStream);

  return true;
}

void SlamWrapperRos::loadParametersAndInitialize() {
  odometryInputPub_ = nh_->advertise<sensor_msgs::PointCloud2>("odom_input", 1, true);
  mappingInputPub_ = nh_->advertise<sensor_msgs::PointCloud2>("mapping_input", 1, true);
  assembledMapPub_ = nh_->advertise<sensor_msgs::PointCloud2>("assembled_map", 1, true);
  denseMapPub_ = nh_->advertise<sensor_msgs::PointCloud2>("dense_map", 1, true);

  submapsPub_ = nh_->advertise<sensor_msgs::PointCloud2>("submaps", 1, true);
  submapOriginsPub_ = nh_->advertise<visualization_msgs::MarkerArray>("submap_origins", 1, true);

  saveMapSrv_ = nh_->advertiseService("save_map", &SlamWrapperRos::saveMapCallback, this);
  saveSubmapsSrv_ = nh_->advertiseService("save_submaps", &SlamWrapperRos::saveSubmapsCallback, this);

  scan2scanTransformPublisher_ = nh_->advertise<geometry_msgs::TransformStamped>("scan2scan_transform", 1, true);
  scan2scanOdomPublisher_ = nh_->advertise<nav_msgs::Odometry>("scan2scan_odometry", 1, true);
  scan2mapTransformPublisher_ = nh_->advertise<geometry_msgs::TransformStamped>("scan2map_transform", 1, true);
  scan2mapOdomPublisher_ = nh_->advertise<nav_msgs::Odometry>("scan2map_odometry", 1, true);

  trackedPathPub_ = nh_->advertise<nav_msgs::Path>("tracked_path_live", 1, true);
  bestGuessPathPub_ = nh_->advertise<nav_msgs::Path>("best_guess_path_live", 1, true);

  //	auto &logger = open3d::utility::Logger::GetInstance();
  //	logger.SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);
  const bool isOfflineReplay = o3d_slam::tryGetParam<bool>("is_read_from_rosbag", *nh_);

  folderPath_ = ros::package::getPath("open3d_slam_ros") + "/data/";
  mapSavingFolderPath_ = nh_->param<std::string>("map_saving_folder", folderPath_);

  exportIMUdata_ = nh_->param<bool>("export_imu_data", false);
  useSyncedPoses_ = nh_->param<bool>("use_syncronized_poses_to_replay", false);
  bagReplayStartTime_ = nh_->param<double>("replay_start_time_as_second", 0.0);
  bagReplayEndTime_ = nh_->param<double>("replay_end_time_as_second", 8000.0);
  asyncOdometryTopic_ = nh_->param<std::string>("async_pose_topic", "/state_estimator/pose_in_odom");

  frames_.rangeSensorFrame = nh_->param<std::string>("tracked_sensor_frame", "default");

  frames_.assumed_external_odometry_tracked_frame = nh_->param<std::string>("assumed_external_odometry_tracked_frame", "default");
  
  if (isOfflineReplay){
    ROS_INFO_STREAM("\033[92m" << "The assumed external odometry tracked frame is: " << frames_.assumed_external_odometry_tracked_frame << "\033[0m");
    ROS_INFO_STREAM("\033[92m" << "The tracked sensor frame and the expected cloud header frame is: " << frames_.rangeSensorFrame << "\033[0m");
    ROS_INFO_STREAM( "Replay Time Config: Start Time(s): " << bagReplayStartTime_ << " End Time(s): " << bagReplayEndTime_);
  }
  
  // Set and load the libpointmatcher config here.
  std::string libpointmatcherConfigPath = ros::package::getPath("open3d_slam_ros") + "/param/icp.yaml";
  ROS_INFO_STREAM("libpointmatcherConfigPath: " << libpointmatcherConfigPath);

  const std::string paramFolderPath = nh_->param<std::string>("parameter_folder_path", "");
  const std::string paramFilename = nh_->param<std::string>("parameter_filename", "");
  SlamParameters params;
  io_lua::loadParameters(paramFolderPath, paramFilename, &params_);
  BASE::loadParametersAndInitialize();

  if(!readLibpointmatcherConfig(libpointmatcherConfigPath)){
    std::cout << "Returning early couldnt load ICP params for libpointmatcher " << std::endl;
    return;
  }
}

bool SlamWrapperRos::saveMapCallback(open3d_slam_msgs::SaveMap::Request& req, open3d_slam_msgs::SaveMap::Response& res) {
  const bool savingResult = saveMap(mapSavingFolderPath_);
  res.statusMessage = savingResult ? "Map saved to: " + mapSavingFolderPath_ : "Error while saving map";
  return true;
}
bool SlamWrapperRos::saveSubmapsCallback(open3d_slam_msgs::SaveSubmaps::Request& req, open3d_slam_msgs::SaveSubmaps::Response& res) {
  const bool savingResult = saveSubmaps(mapSavingFolderPath_);
  res.statusMessage = savingResult ? "Submaps saved to: " + mapSavingFolderPath_ : "Error while saving submaps";
  return true;
}

void SlamWrapperRos::publishMapToOdomTf(const Time& time) {
  const ros::Time timestamp = toRos(time);
  o3d_slam::publishTfTransform(mapper_->getMapToOdom(time).matrix(), timestamp, frames_.mapFrame, frames_.odomFrame, tfBroadcaster_.get());
  o3d_slam::publishTfTransform(mapper_->getMapToRangeSensor(time).matrix(), timestamp, frames_.mapFrame, "raw_rs_o3d", tfBroadcaster_.get());

  if (!(mapper_->getMapToRangeSensorBuffer().empty()))
  {
    auto latestMapToRangeMeasurement_ = mapper_->getMapToRangeSensorBuffer().latest_measurement();

    if ((isTimeValid(latestMapToRangeMeasurement_.time_))){
      // Publish lidar to map transform.
      o3d_slam::publishTfTransform(latestMapToRangeMeasurement_.transform_.matrix().inverse(), o3d_slam::toRos(latestMapToRangeMeasurement_.time_), frames_.rangeSensorFrame, frames_.mapFrame, tfBroadcaster_.get());
    }
  }
}

void SlamWrapperRos::publishDenseMap(const Time& time) {
  if (denseMapVisualizationUpdateTimer_.elapsedMsec() < params_.visualization_.visualizeEveryNmsec_) {
    return;
  }
  const auto denseMap = mapper_->getActiveSubmap().getDenseMapCopy();  // copy
  const ros::Time timestamp = toRos(time);
  o3d_slam::publishCloud(denseMap.toPointCloud(), frames_.mapFrame, timestamp, denseMapPub_);
}

void SlamWrapperRos::publishMaps(const Time& time) {
  if (visualizationUpdateTimer_.elapsedMsec() < params_.visualization_.visualizeEveryNmsec_ && !isVisualizationFirstTime_) {
    return;
  }

  const ros::Time timestamp = toRos(time);
  {
    PointCloud map = mapper_->getAssembledMapPointCloud();
    voxelize(params_.visualization_.assembledMapVoxelSize_, &map);
    o3d_slam::publishCloud(map, frames_.mapFrame, timestamp, assembledMapPub_);
  }
  o3d_slam::publishCloud(mapper_->getPreprocessedScan(), frames_.rangeSensorFrame, timestamp, mappingInputPub_);
  o3d_slam::publishSubmapCoordinateAxes(mapper_->getSubmaps(), frames_.mapFrame, timestamp, submapOriginsPub_);
  if (submapsPub_.getNumSubscribers() > 0) {
    open3d::geometry::PointCloud cloud;
    o3d_slam::assembleColoredPointCloud(mapper_->getSubmaps(), &cloud);
    voxelize(params_.visualization_.submapVoxelSize_, &cloud);
    o3d_slam::publishCloud(cloud, frames_.mapFrame, timestamp, submapsPub_);
  }

  visualizationUpdateTimer_.reset();
  isVisualizationFirstTime_ = false;
}

}  // namespace o3d_slam
