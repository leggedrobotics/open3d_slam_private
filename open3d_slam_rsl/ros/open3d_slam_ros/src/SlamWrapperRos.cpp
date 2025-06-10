#include "open3d_slam_ros/SlamWrapperRos.hpp"

#include <open3d/Open3D.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <mutex>
#include <memory>
#include <thread>

#include "open3d_conversions/open3d_conversions.hpp"
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

SlamWrapperRos::SlamWrapperRos(const rclcpp::Node::SharedPtr& nh)
    : BASE(), nh_(nh) {
  tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
  staticTfBroadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(nh_);

  prevPublishedTimeScanToScan_ = fromUniversal(0);
  prevPublishedTimeScanToMap_ = fromUniversal(0);
}

SlamWrapperRos::~SlamWrapperRos() {
  if (tfWorkerThread_.joinable()) tfWorkerThread_.join();
  if (visualizationWorkerThread_.joinable()) visualizationWorkerThread_.join();
  if (params_.odometry_.isPublishOdometryMsgs_ && odomPublisherWorkerThread_.joinable())
    odomPublisherWorkerThread_.join();
}

void SlamWrapperRos::startWorkers() {
  tfWorkerThread_ = std::thread([this]() { tfWorker(); });
  visualizationWorkerThread_ = std::thread([this]() { visualizationWorker(); });
  if (params_.odometry_.isPublishOdometryMsgs_)
    odomPublisherWorkerThread_ = std::thread([this]() { odomPublisherWorker(); });

  BASE::startWorkers();
}

void SlamWrapperRos::odomPublisherWorker() {
  rclcpp::Rate r(100.0);
  while (rclcpp::ok()) {
    auto getTransformMsg = [this](const Transform& T, const Time& t) {
      rclcpp::Time timestamp = toRos(t);
      geometry_msgs::msg::TransformStamped transformMsg = o3d_slam::toRos(T.matrix(), timestamp, frames_.mapFrame, frames_.rangeSensorFrame);
      return transformMsg;
    };

    auto getOdomMsg = [](const geometry_msgs::msg::TransformStamped& transformMsg) {
      nav_msgs::msg::Odometry odomMsg;
      odomMsg.header = transformMsg.header;
      odomMsg.child_frame_id = transformMsg.child_frame_id;
      odomMsg.pose.pose.orientation = transformMsg.transform.rotation;
      odomMsg.pose.pose.position.x = transformMsg.transform.translation.x;
      odomMsg.pose.pose.position.y = transformMsg.transform.translation.y;
      odomMsg.pose.pose.position.z = transformMsg.transform.translation.z;
      return odomMsg;
    };

    const Time latestScanToMap = latestScanToMapRefinementTimestamp_;
    const bool isScanToMapAlreadyPublished = latestScanToMap == prevPublishedTimeScanToMap_;
    if (!isScanToMapAlreadyPublished && mapper_->hasProcessedMeasurements()) {
      const Transform T = mapper_->getMapToRangeSensor(latestScanToMap);
      geometry_msgs::msg::TransformStamped transformMsg = getTransformMsg(T, latestScanToMap);
      nav_msgs::msg::Odometry odomMsg = getOdomMsg(transformMsg);
      if (scan2mapTransformPublisher_ && scan2mapTransformPublisher_->get_subscription_count() > 0)
        scan2mapTransformPublisher_->publish(transformMsg);

      odomMsg.child_frame_id = frames_.rangeSensorFrame;
      if (scan2mapOdomPublisher_ && scan2mapOdomPublisher_->get_subscription_count() > 0)
        scan2mapOdomPublisher_->publish(odomMsg);

      prevPublishedTimeScanToMap_ = latestScanToMap;
    }
    r.sleep();
  }
}

void SlamWrapperRos::tfWorker() {
  rclcpp::Rate r(500.0);
  while (rclcpp::ok()) {
    const Time latestScanToScan = latestScanToScanRegistrationTimestamp_;
    const bool isAlreadyPublished = (latestScanToScan == prevPublishedTimeScanToScan_);
    if (!isAlreadyPublished && odometry_ && odometry_->hasProcessedMeasurements()) {
      const Transform T = odometry_->getOdomToRangeSensor(latestScanToScan);
      rclcpp::Time timestamp = toRos(latestScanToScan);
      o3d_slam::publishTfTransform(T.matrix().inverse(), timestamp, frames_.rangeSensorFrame, frames_.odomFrame, tfBroadcaster_.get());
      prevPublishedTimeScanToScan_ = latestScanToScan;
    }

    const Time latestScanToMap = latestScanToMapRefinementTimestamp_;

    if (!isTimeValid(latestScanToMap)) {
      r.sleep();
      continue;
    }
    const bool isScanToMapAlreadyPublished = (latestScanToMap == prevPublishedTimeScanToMap_);
    if (!isScanToMapAlreadyPublished && mapper_ && mapper_->hasProcessedMeasurements()) {
      publishMapToOdomTf(latestScanToMap);
      prevPublishedTimeScanToMap_ = latestScanToMap;

      nav_msgs::msg::Path trackedPathCopy;
      nav_msgs::msg::Path bestGuessPathCopy;
      {
        std::lock_guard<std::mutex> lock(mapper_->pathMutex_);
        trackedPathCopy = mapper_->trackedPath_;
        bestGuessPathCopy = mapper_->bestGuessPath_;
      }

      if (trackedPathPub_ && trackedPathPub_->get_subscription_count() > 0 && isPathValid(trackedPathCopy)) {
        trackedPathCopy.header.stamp = o3d_slam::toRos(latestScanToMap);
        trackedPathCopy.header.frame_id = frames_.mapFrame;
        trackedPathPub_->publish(trackedPathCopy);
      }

      if (bestGuessPathPub_ && bestGuessPathPub_->get_subscription_count() > 0 && isPathValid(bestGuessPathCopy)) {
        bestGuessPathCopy.header.stamp = o3d_slam::toRos(latestScanToMap);
        bestGuessPathCopy.header.frame_id = frames_.mapFrame;
        bestGuessPathPub_->publish(bestGuessPathCopy);
      }

      if (!trackedPathCopy.poses.empty() && !bestGuessPathCopy.poses.empty()) {
        drawLinesBetweenPoses(trackedPathCopy, bestGuessPathCopy, toRos(latestScanToMap));
      }
    }
    r.sleep();
  }
}


void SlamWrapperRos::offlineTfWorker() {
  const Time latestScanToScan = latestScanToScanRegistrationTimestamp_;
  const bool isAlreadyPublished = latestScanToScan == prevPublishedTimeScanToScan_;
  if ((!isAlreadyPublished) && (odometry_->hasProcessedMeasurements())) {
    const Transform T = odometry_->getOdomToRangeSensor(latestScanToScan);
    rclcpp::Time timestamp = toRos(latestScanToScan);
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


bool SlamWrapperRos::isPathValid(const nav_msgs::msg::Path& path) const {
  if (path.poses.empty()) return false;
  for (const auto& pose : path.poses) {
    if (pose.header.frame_id.empty()) return false;
    const auto& p = pose.pose.position;
    const auto& q = pose.pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) return false;
  }
  return true;
}

void SlamWrapperRos::drawLinesBetweenPoses(const nav_msgs::msg::Path& path1, const nav_msgs::msg::Path& path2, const rclcpp::Time& stamp) {
  if (!differenceLinePub_ || differenceLinePub_->get_subscription_count() == 0) return;
  if (path1.poses.size() != path2.poses.size()) {
    RCLCPP_ERROR(nh_->get_logger(), "Path sizes are not equal. Skipping the line drawing.");
    return;
  }

  visualization_msgs::msg::Marker line_list;
  line_list.header.frame_id = frames_.mapFrame;
  line_list.header.stamp = stamp;
  line_list.ns = "paths";
  line_list.action = visualization_msgs::msg::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 0;
  line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_list.scale.x = 0.006;

  line_list.color.r = 0.0;
  line_list.color.g = 1.0;
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;

  for (size_t i = 0; i < path1.poses.size(); i++) {
    geometry_msgs::msg::Point p_start;
    p_start.x = path1.poses[i].pose.position.x;
    p_start.y = path1.poses[i].pose.position.y;
    p_start.z = path1.poses[i].pose.position.z;
    line_list.points.push_back(p_start);

    geometry_msgs::msg::Point p_end;
    p_end.x = path2.poses[i].pose.position.x;
    p_end.y = path2.poses[i].pose.position.y;
    p_end.z = path2.poses[i].pose.position.z;
    line_list.points.push_back(p_end);
  }

  differenceLinePub_->publish(line_list);
}

void SlamWrapperRos::offlineVisualizationWorker() {
  const Time scanToScanTimestamp = latestScanToScanRegistrationTimestamp_;
  rclcpp::Time timestamp = toRos(scanToScanTimestamp);
  o3d_slam::publishSubmapCoordinateAxes(mapper_->getSubmaps(), frames_.mapFrame, timestamp, submapOriginsPub_);
}

void SlamWrapperRos::visualizationWorker() {
  rclcpp::Rate r(20.0);
  while (rclcpp::ok()) {
    // const Time scanToScanTimestamp = latestScanToScanRegistrationTimestamp_;
    // if (odometryInputPub_ && odometryInputPub_->get_subscription_count() > 0 && isTimeValid(scanToScanTimestamp)) {
    //   const PointCloud odomInput = odometry_->getPreProcessedCloud();
    //   o3d_slam::publishCloud(odomInput, frames_.rangeSensorFrame, toRos(scanToScanTimestamp), odometryInputPub_);
    // }

    const Time scanToMapTimestamp = latestScanToMapRefinementTimestamp_;
    if (isTimeValid(scanToMapTimestamp)) {
      publishDenseMap(scanToMapTimestamp);
      publishMaps(scanToMapTimestamp);
      if (!params_.mapper_.republishMap_ && params_.mapper_.isUseInitialMap_) break;
    }
    r.sleep();
  }
}

bool SlamWrapperRos::readLibpointmatcherConfig(const std::string& path) {
  // If you want to load libpointmatcher config in ROS2, port this logic accordingly.
  return true;
}

void SlamWrapperRos::loadParametersAndInitialize() {
// odometryInputPub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom_input", 1);
  mappingInputPub_    = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("mapping_input", 1);
  assembledMapPub_    = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("assembled_map", 1);
  denseMapPub_        = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("dense_map", 1);
  submapsPub_         = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("submaps", 1);
  submapOriginsPub_   = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("submap_origins", 1);

  saveMapSrv_ = nh_->create_service<open3d_slam_msgs::srv::SaveMap>(
      "save_map", std::bind(&SlamWrapperRos::saveMapCallback, this, std::placeholders::_1, std::placeholders::_2));
  saveSubmapsSrv_ = nh_->create_service<open3d_slam_msgs::srv::SaveSubmaps>(
      "save_submaps", std::bind(&SlamWrapperRos::saveSubmapsCallback, this, std::placeholders::_1, std::placeholders::_2));

  scan2scanTransformPublisher_ = nh_->create_publisher<geometry_msgs::msg::TransformStamped>("scan2scan_transform", 1);
  scan2scanOdomPublisher_      = nh_->create_publisher<nav_msgs::msg::Odometry>("scan2scan_odometry", 1);
  scan2mapTransformPublisher_  = nh_->create_publisher<geometry_msgs::msg::TransformStamped>("scan2map_transform", 1);
  scan2mapOdomPublisher_       = nh_->create_publisher<nav_msgs::msg::Odometry>("scan2map_odometry", 10);

  trackedPathPub_ = nh_->create_publisher<nav_msgs::msg::Path>("tracked_path_live", 1);
  bestGuessPathPub_ = nh_->create_publisher<nav_msgs::msg::Path>("best_guess_path_live", 1);
  differenceLinePub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("differenceLines", 1);

  folderPath_ = nh_->declare_parameter<std::string>("data_folder", "") + "/data/";
  mapSavingFolderPath_ = nh_->declare_parameter<std::string>("map_saving_folder", folderPath_);

  exportIMUdata_ = nh_->declare_parameter<bool>("export_imu_data", false);
  useSyncedPoses_ = nh_->declare_parameter<bool>("use_syncronized_poses_to_replay", false);
  rePublishTf_ = nh_->declare_parameter<bool>("republish_tf_topic", false);

  relativeSleepDuration_ = nh_->declare_parameter<double>("relative_sleep_duration", 0.0);
  bagReplayStartTime_ = nh_->declare_parameter<double>("replay_start_time_as_second", 0.0);
  bagReplayEndTime_ = nh_->declare_parameter<double>("replay_end_time_as_second", 8000.0);
  asyncOdometryTopic_ = nh_->declare_parameter<std::string>("async_pose_topic", "/state_estimator/pose_in_odom");

  frames_.rangeSensorFrame = "default";
  frames_.assumed_external_odometry_tracked_frame = nh_->declare_parameter<std::string>("assumed_external_odometry_tracked_frame", "default");

  if (!nh_->has_parameter("parameter_folder_path")) {
    nh_->declare_parameter<std::string>("parameter_folder_path", "");
  }
  if (!nh_->has_parameter("parameter_filename")) {
    nh_->declare_parameter<std::string>("parameter_filename", "");
  }
  const std::string paramFolderPath = nh_->get_parameter("parameter_folder_path").as_string();
  const std::string paramFilename   = nh_->get_parameter("parameter_filename").as_string();


  
  SlamParameters params;
  io_lua::loadParameters(paramFolderPath, paramFilename, &params_);
  BASE::loadParametersAndInitialize();

  readLibpointmatcherConfig(nh_->declare_parameter<std::string>("icp_yaml_path", "/path/to/icp.yaml"));
}

bool SlamWrapperRos::saveMapCallback(
    const std::shared_ptr<open3d_slam_msgs::srv::SaveMap::Request> req,
    std::shared_ptr<open3d_slam_msgs::srv::SaveMap::Response> res) {
  const bool savingResult = saveMap(mapSavingFolderPath_);
  res->status_message = savingResult ? "Map saved to: " + mapSavingFolderPath_ : "Error while saving map";
  return true;
}

bool SlamWrapperRos::saveSubmapsCallback(
    const std::shared_ptr<open3d_slam_msgs::srv::SaveSubmaps::Request> req,
    std::shared_ptr<open3d_slam_msgs::srv::SaveSubmaps::Response> res) {
  const bool savingResult = saveSubmaps(mapSavingFolderPath_);
  res->status_message = savingResult ? "Submaps saved to: " + mapSavingFolderPath_ : "Error while saving submaps";
  return true;
}

void SlamWrapperRos::publishMapToOdomTf(const Time& time) {
  const rclcpp::Time timestamp = toRos(time);
  o3d_slam::publishTfTransform(mapper_->getMapToRangeSensor(time).matrix(), timestamp, frames_.mapFrame, "raw_rs_o3d", tfBroadcaster_.get());

  if (!(mapper_->getMapToRangeSensorBuffer().empty())) {
    auto latestMapToRangeMeasurement_ = mapper_->getMapToRangeSensorBuffer().latest_measurement();

    if ((isTimeValid(latestMapToRangeMeasurement_.time_))) {
      std::string adaptedMapFrame = frames_.mapFrame;
      o3d_slam::publishTfTransform(latestMapToRangeMeasurement_.transform_.matrix().inverse(),
                                   o3d_slam::toRos(latestMapToRangeMeasurement_.time_), frames_.rangeSensorFrame, adaptedMapFrame,
                                   tfBroadcaster_.get());
    }
  }
}

void SlamWrapperRos::publishDenseMap(const Time& time) {
  if (denseMapVisualizationUpdateTimer_.elapsedMsec() < params_.visualization_.visualizeEveryNmsec_) return;
  const auto denseMap = mapper_->getActiveSubmap().getDenseMapCopy();  // copy
  const rclcpp::Time timestamp = toRos(time);
  o3d_slam::publishCloud(denseMap.toPointCloud(), frames_.mapFrame, timestamp, denseMapPub_);
}

void SlamWrapperRos::publishMaps(const Time& time) {
  if (visualizationUpdateTimer_.elapsedMsec() < params_.visualization_.visualizeEveryNmsec_ && !isVisualizationFirstTime_) return;

  const rclcpp::Time timestamp = toRos(time);
  {
    if (assembledMapPub_ && assembledMapPub_->get_subscription_count() > 0) {
      PointCloud map = mapper_->getAssembledMapPointCloudVisualization();
      o3d_slam::publishCloud(map, frames_.mapFrame, timestamp, assembledMapPub_);
    }
  }
  if (mappingInputPub_ && mappingInputPub_->get_subscription_count() > 0) {
    o3d_slam::publishCloud(mapper_->getPreprocessedScan(), frames_.rangeSensorFrame, timestamp, mappingInputPub_);
  }
  if (submapOriginsPub_ && submapOriginsPub_->get_subscription_count() > 0) {
    o3d_slam::publishSubmapCoordinateAxes(mapper_->getSubmaps(), frames_.mapFrame, timestamp, submapOriginsPub_);
  }
  if (submapsPub_ && submapsPub_->get_subscription_count() > 0) {
    open3d::geometry::PointCloud cloud;
    o3d_slam::assembleColoredPointCloud(mapper_->getSubmaps(), &cloud);
    voxelize(params_.visualization_.submapVoxelSize_, &cloud);
    o3d_slam::publishCloud(cloud, frames_.mapFrame, timestamp, submapsPub_);
  }
  visualizationUpdateTimer_.reset();
  isVisualizationFirstTime_ = false;
}

}  // namespace o3d_slam
