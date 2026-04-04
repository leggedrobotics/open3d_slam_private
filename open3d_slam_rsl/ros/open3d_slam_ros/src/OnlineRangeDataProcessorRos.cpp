/*
 * OnlineRangeDataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/OnlineRangeDataProcessorRos.hpp"
#include <chrono>
#include <ros/master.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam/math.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"
namespace o3d_slam {

OnlineRangeDataProcessorRos::OnlineRangeDataProcessorRos(ros::NodeHandlePtr nh) : BASE(nh), tfListener_(tfBuffer_) {}

void OnlineRangeDataProcessorRos::initialize() {
  initCommonRosStuff();
  slam_ = std::make_shared<SlamWrapperRos>(nh_);
  imuBufferPtr_ = std::make_shared<ImuBuffer>();
  // If this is calling the ros wrapper version. Which overrides the base. The base function is later called within the overridden version.
  slam_->loadParametersAndInitialize();
}

bool OnlineRangeDataProcessorRos::readCalibrationIfNeeded() {
  if (slam_->frames_.rangeSensorFrame == "default") {
    ROS_WARN_STREAM_THROTTLE(
        2.0, "Range sensor frame is not set yet (cloud didnt arrive yet). Delaying the transformation look-up. (Throttled 2s)");
    return false;
  }

  // The frames are identical. This is often not the case since the odometry follows a certain frame like base but the point clouds arrive
  // in the lidar frame.
  if ((slam_->frames_.rangeSensorFrame == slam_->frames_.assumed_external_odometry_tracked_frame) || !slam_->isUsingOdometryTopic()) {
    slam_->setExternalOdometryFrameToCloudFrameCalibration(Eigen::Isometry3d::Identity());
    return true;
  }

  if (!isStaticTransformFound_) {
    try {
      // Waits for the transform to be available. After this we dont need to have timeout for the lookup itself
      if (!tfBuffer_.canTransform(slam_->frames_.rangeSensorFrame, slam_->frames_.assumed_external_odometry_tracked_frame, ros::Time(0.0),
                                  ros::Duration(0.2))) {
        ROS_WARN_STREAM_THROTTLE(0.5, "Transform not available yet: [" << slam_->frames_.rangeSensorFrame << "] to ["
                                                                       << slam_->frames_.assumed_external_odometry_tracked_frame
                                                                       << "]. Thottled 0.5s");
        return false;
      }

      auto T_L_sensorFrame = tfBuffer_.lookupTransform(
          slam_->frames_.rangeSensorFrame, slam_->frames_.assumed_external_odometry_tracked_frame, ros::Time(0.0), ros::Duration(0.0));

      ROS_INFO_STREAM("\033[92m"
                      << "Found the transform between " << slam_->frames_.rangeSensorFrame << " and "
                      << slam_->frames_.assumed_external_odometry_tracked_frame << "\033[0m");
      ROS_INFO_STREAM("\033[92m"
                      << "You dont believe me? Here it is:\n " << T_L_sensorFrame << "\033[0m");

      // Set the frame transformation between the external odometry frame and the range sensor frame.
      slam_->setExternalOdometryFrameToCloudFrameCalibration(tf2::transformToEigen(T_L_sensorFrame));

      if (slam_->isIMUattitudeInitializationEnabled()) {
        // Waits for the transform to be available. After this we dont need to have timeout for the lookup itself
        if (!tfBuffer_.canTransform(slam_->frames_.rangeSensorFrame, slam_->frames_.imuFrame, ros::Time(0.0), ros::Duration(0.2))) {
          ROS_WARN_STREAM("Transform not available yet: [" << slam_->frames_.rangeSensorFrame << "] to [" << slam_->frames_.imuFrame
                                                           << "].");
          return false;
        }

        // <X frame to Y frame> for the tf look up.
        auto RangeSensorFrameToimuFrame =
            tfBuffer_.lookupTransform(slam_->frames_.rangeSensorFrame, slam_->frames_.imuFrame, ros::Time(0.0), ros::Duration(0.0));

        ROS_INFO_STREAM("\033[92m"
                        << "Found the transform between " << slam_->frames_.rangeSensorFrame << " and " << slam_->frames_.imuFrame
                        << "\033[0m");
        ROS_INFO_STREAM("\033[92m"
                        << "You dont believe me? Here it is:\n " << RangeSensorFrameToimuFrame << "\033[0m");

        // Set the frame transformation between the external odometry frame and the range sensor frame.
        lidarToImu_.matrix() = tf2::transformToEigen(RangeSensorFrameToimuFrame).matrix();  //.inverse();
      }
      return true;

    } catch (const tf2::TransformException& exception) {
      ROS_WARN_STREAM("Caught exception while looking for the transform frame: " << slam_->frames_.rangeSensorFrame << " to "
                                                                                 << slam_->frames_.assumed_external_odometry_tracked_frame
                                                                                 << "." << exception.what());
      return false;
    }
  } else {
    ROS_WARN_STREAM("This is unexpected, something is off.");
    return false;
  }
}

void OnlineRangeDataProcessorRos::dynamicPoseDiscoveryCallback(const ros::TimerEvent&) {
  if (poseSubscribed_) {
    dynamicPoseDiscoveryTimer_.stop();
    return;
  }

  std::vector<ros::master::TopicInfo> topics;
  ros::master::getTopics(topics);

  auto topicExists = [&topics](const std::string& name) -> bool {
    for (const auto& t : topics) {
      if (t.name == name) return true;
    }
    return false;
  };

  if (topicExists(poseStampedWithCovarianceTopic_)) {
    poseStampedCovarianceSubscriber_ =
        nh_->subscribe(poseStampedWithCovarianceTopic_, poseSubscriberQueueSize_,
                       &OnlineRangeDataProcessorRos::poseStampedWithCovarianceCallback, this, ros::TransportHints().tcpNoDelay());
    poseSubscribed_ = true;
    ROS_INFO_STREAM("\033[92m"
                    << "Dynamically subscribed to poseStampedWithCovariance topic: " << poseStampedWithCovarianceTopic_ << "\033[0m");
  } else if (topicExists(poseStampedTopic_)) {
    poseStampedSubscriber_ = nh_->subscribe(poseStampedTopic_, poseSubscriberQueueSize_,
                                            &OnlineRangeDataProcessorRos::poseStampedCallback, this,
                                            ros::TransportHints().tcpNoDelay());
    poseSubscribed_ = true;
    ROS_INFO_STREAM("\033[92m"
                    << "Dynamically subscribed to poseStamped topic: " << poseStampedTopic_ << "\033[0m");
  } else if (topicExists(odometryTopic_)) {
    odometrySubscriber_ = nh_->subscribe(odometryTopic_, poseSubscriberQueueSize_, &OnlineRangeDataProcessorRos::odometryCallback, this,
                                         ros::TransportHints().tcpNoDelay());
    poseSubscribed_ = true;
    ROS_INFO_STREAM("\033[92m"
                    << "Dynamically subscribed to odometry topic: " << odometryTopic_ << "\033[0m");
  }
}

void OnlineRangeDataProcessorRos::startProcessing() {
  slam_->startWorkers();

  cloudSubscriberQueueSize_ = static_cast<uint32_t>(std::max(1, nh_->param<int>("cloud_subscriber_queue_size", 1)));
  poseSubscriberQueueSize_ = static_cast<uint32_t>(std::max(1, nh_->param<int>("pose_subscriber_queue_size", 10)));

  cloudSubscriber_ =
      nh_->subscribe(cloudTopic_, cloudSubscriberQueueSize_, &OnlineRangeDataProcessorRos::cloudCallback, this,
                     ros::TransportHints().tcpNoDelay());

  if (slam_->isIMUattitudeInitializationEnabled()) {
    imuSubscriber_ = nh_->subscribe<sensor_msgs::Imu>(imuTopic_, 40, &OnlineRangeDataProcessorRos::imuCallback, this,
                                                      ros::TransportHints().tcpNoDelay());
    ROS_INFO_STREAM("Subscribed to IMU topic: " << imuTopic_);
  } else {
    isAttitudeInitialized_ = true;
  }

  staticTfCallback_ = nh_->createTimer(ros::Duration(0.1), &OnlineRangeDataProcessorRos::staticTfCallback, this);
  dynamicPoseDiscoveryTimer_ = nh_->createTimer(ros::Duration(0.5), &OnlineRangeDataProcessorRos::dynamicPoseDiscoveryCallback, this);

  ROS_INFO("Open3d_slam subscribers initialized. Waiting for pose topic...");

  unsigned int n_threads = std::max(2u, std::min(4u, std::thread::hardware_concurrency()));
  ros::AsyncSpinner spinner(n_threads);
  spinner.start();

  ros::waitForShutdown();

  slam_->stopWorkers();
}

void OnlineRangeDataProcessorRos::staticTfCallback(const ros::TimerEvent&) {
  if (!slam_->isUsingOdometryTopic()) {
    slam_->setExternalOdometryFrameToCloudFrameCalibration(Eigen::Isometry3d::Identity());
    tryProcessPendingClouds();
    publishCompletedMappingResultIfAvailable();
    staticTfCallback_.stop();
  }

  if (readCalibrationIfNeeded()) {
    // If IMU initialization is enabled we need to wait for the IMU callback to initialize the attitude.
    if (!slam_->isIMUattitudeInitializationEnabled()) {
      // // This casts isometry3d to affine3d.
      Eigen::Isometry3d T_L_sensorFrame = slam_->getExternalOdometryFrameToCloudFrameCalibration();
      // const auto latest_T_W_sensorFrame = slam_->getLatestOdometryPoseMeasurement();
      // // Actual transformation applied to the odometry measurement. Reads as pose of Lidar frame in the external odometry frame.
      // Eigen::Isometry3d T_M_L = latest_T_W_sensorFrame.transform_ * T_L_sensorFrame.inverse();

      if (!slam_->isUseExistingMapEnabled()) {
        slam_->setInitialTransform(T_L_sensorFrame.inverse().matrix());
      }
    }

    tryProcessPendingClouds();
    publishCompletedMappingResultIfAvailable();
    ROS_INFO("Static TF reader callback is terminated after successfully reading the transform.");
    staticTfCallback_.stop();
  }
}

void OnlineRangeDataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {
  enqueueMeasurement(cloud, timestamp, std::chrono::steady_clock::now());
}

void OnlineRangeDataProcessorRos::enqueueMeasurement(const PointCloud& cloud, const Time& timestamp,
                                                     const std::chrono::steady_clock::time_point& ingressWallTime) {
  slam_->recordCloudIngressWallTime(timestamp, ingressWallTime);

  // Re-publish the raw point cloud for visualization purposes.
  o3d_slam::publishCloud(cloud, slam_->frames_.rangeSensorFrame, toRos(timestamp), rawCloudPub_);
  {
    std::lock_guard<std::mutex> lock(pendingCloudsMutex_);
    pendingClouds_.push_back(PendingCloudMeasurement{cloud, timestamp});
  }
  tryProcessPendingClouds();
  publishCompletedMappingResultIfAvailable();
}

std::optional<visualization_msgs::Marker> OnlineRangeDataProcessorRos::generateMarkersForSurfaceNormalVectors(
    const open3d::geometry::PointCloud& pointCloud, const ros::Time& timestamp, const o3d_slam::RgbaColorMap::Values& color) {
  if (pointCloud.IsEmpty()) {
    ROS_WARN("Point cloud is empty.");
    return {};
  }
  if (!pointCloud.HasNormals()) {
    ROS_WARN("Point cloud has no normals");
    return {};
  }

  std_msgs::ColorRGBA colorMsg;
  colorMsg.r = color[0];
  colorMsg.g = color[1];
  colorMsg.b = color[2];
  colorMsg.a = color[3];

  visualization_msgs::Marker vectorsMarker;
  vectorsMarker.header.stamp = timestamp;
  vectorsMarker.header.frame_id = slam_->frames_.rangeSensorFrame;
  vectorsMarker.ns = "surface_normals";
  vectorsMarker.action = visualization_msgs::Marker::ADD;
  vectorsMarker.type = visualization_msgs::Marker::LINE_LIST;
  vectorsMarker.pose.orientation.w = 1.0;
  vectorsMarker.id = 0;
  vectorsMarker.scale.x = 0.02;
  vectorsMarker.color = colorMsg;

  const size_t n = pointCloud.points_.size();
  vectorsMarker.points.resize(n * 2);

  const auto& points = pointCloud.points_;
  const auto& normals = pointCloud.normals_;

#pragma omp parallel for
  for (int i = 0; i < static_cast<int>(n); ++i) {
    // Start point of the normal
    vectorsMarker.points[2 * i].x = points[i][0];
    vectorsMarker.points[2 * i].y = points[i][1];
    vectorsMarker.points[2 * i].z = points[i][2];

    // End point (arrow tip)
    vectorsMarker.points[2 * i + 1].x = points[i][0] + normals[i][0] * 0.09;
    vectorsMarker.points[2 * i + 1].y = points[i][1] + normals[i][1] * 0.09;
    vectorsMarker.points[2 * i + 1].z = points[i][2] + normals[i][2] * 0.09;
  }

  return vectorsMarker;
}

void OnlineRangeDataProcessorRos::processOdometry(const Transform& transform, const Time& timestamp) {
  if (!slam_->isUsingOdometryTopic()) {
    return;
  }

  if (!slam_->addOdometryPoseToBuffer(transform, timestamp)) {
    ROS_ERROR_STREAM("Failed to add odometry pose to buffer. Exiting.");
    return;
  }

  if (!slam_->isIMUattitudeInitializationEnabled()) {
    isAttitudeInitialized_ = true;
  }

  if (!isAttitudeInitialized_) {
    ROS_WARN_STREAM_THROTTLE(1, "Attitude not initialized yet, waiting for IMU measurements. Throttled 1s");
    return;
  }

  tryProcessPendingClouds();
  publishCompletedMappingResultIfAvailable();

  // ROS_DEBUG_STREAM("Processed odometry at time: " << toString(timestamp));
}

void OnlineRangeDataProcessorRos::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  ROS_DEBUG_STREAM("A point cloud has arrived.");
  const auto ingressWallTime = std::chrono::steady_clock::now();
  slam_->frames_.rangeSensorFrame = msg->header.frame_id;
  open3d::geometry::PointCloud cloud;

  if (!open3d_conversions::rosToOpen3d(msg, cloud, false, true)) {
    ROS_ERROR_STREAM("Conversion Failed.");
  }

  const Time timestamp = fromRos(msg->header.stamp);
  enqueueMeasurement(cloud, timestamp, ingressWallTime);
}

void OnlineRangeDataProcessorRos::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_ptr) {
  // Add to buffer
  Eigen::Vector3d linearAcc(imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imu_ptr->angular_velocity.x, imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z);

  Eigen::Matrix<double, 6, 1> addedImuMeasurements;
  addedImuMeasurements = imuBufferPtr_->addToImuBuffer(imu_ptr->header.stamp.toSec(), linearAcc, angularVel);
  publishAddedImuMeas_(addedImuMeasurements, imu_ptr->header.stamp);
  slam_->frames_.imuFrame = imu_ptr->header.frame_id;

  if (isAttitudeInitialized_) {
    return;
  }

  Eigen::Quaterniond initAttitude = Eigen::Quaterniond::Identity();
  Eigen::Vector3d gyrBias = Eigen::Vector3d::Zero();
  double estimatedGravityMagnitude{0.0};
  if (!(imuBufferPtr_->estimateAttitudeFromImu(initAttitude, estimatedGravityMagnitude, gyrBias))) {
    return;
  }

  isAttitudeInitialized_ = true;
  Eigen::Vector3d gravityVector = Eigen::Vector3d(0, 0, 9.80665);
  Eigen::Vector3d estimatedGravityVector = Eigen::Vector3d(0, 0, estimatedGravityMagnitude);
  Eigen::Vector3d gravityVectorError = estimatedGravityVector - gravityVector;
  Eigen::Vector3d gravityVectorErrorInImuFrame = initAttitude.inverse().matrix() * gravityVectorError;
  std::cout << " Gravity error in IMU frame is: " << gravityVectorErrorInImuFrame.transpose() << std::endl;

  if (!slam_->isExternalOdometryFrameToCloudFrameCalibrationSet()) {
    std::cout << " Calibration is not available yet. Returning from IMU attitude initialization. "
              << " \n";
    return;
  }

  // We conciously read from the object to ensure the transformations are nicely set.
  Eigen::Affine3d eigenTransform = slam_->getExternalOdometryFrameToCloudFrameCalibration();
  geometry_msgs::TransformStamped calibrationAsTransform = tf2::eigenToTransform(eigenTransform);

  // Bookkeeping
  geometry_msgs::PoseStamped odomPose_transformed;
  geometry_msgs::PoseStamped odomPose;

  const auto latestOdomMeasurement = slam_->getLatestOdometryPoseMeasurement();
  odomPose.pose = o3d_slam::getPose(latestOdomMeasurement.transform_.matrix());

  // Actual transformation applied to the odometry measurement. Reads as pose of Lidar frame in the external odometry frame.
  tf2::doTransform(odomPose, odomPose_transformed, calibrationAsTransform);

  // Here we dont want to use the orientation of the odometry measurement. We will acquire it from IMU anyway.
  odomPose_transformed.pose.orientation.w = 1.0;
  odomPose_transformed.pose.orientation.z = 0.0;
  odomPose_transformed.pose.orientation.y = 0.0;
  odomPose_transformed.pose.orientation.x = 0.0;

  // Convert the attitude of the IMU to the attitude of the LiDAR.
  Transform initAttitudeOfLiDAR = initAttitude * lidarToImu_.inverse();

  std::cout << " The initial pose of LiDAR is: "
            << "\033[92m" << o3d_slam::asString(initAttitudeOfLiDAR) << " \n";

  // This casts isometry3d to affine3d.
  Transform newTransform = o3d_slam::getTransform(odomPose_transformed.pose) * initAttitudeOfLiDAR;

  // initialTransform.affine().matrix().block<3, 3>(0, 0) = initAttitudeOfLiDAR.affine().matrix().block<3, 3>(0, 0);
  slam_->setInitialTransform(newTransform.matrix());
  tryProcessPendingClouds();
  publishCompletedMappingResultIfAvailable();
}

void OnlineRangeDataProcessorRos::publishAddedImuMeas_(const Eigen::Matrix<double, 6, 1>& addedImuMeas, const ros::Time& stamp) {
  // Publish added imu measurement
  if (addedImuMeasPub_.getNumSubscribers() == 0 && !addedImuMeasPub_.isLatched()) {
    // Early Return since IMU is at 400hz. We dont want to publish this if no one is listening.
    return;
  }

  sensor_msgs::Imu addedImuMeasMsg;
  addedImuMeasMsg.header.stamp = stamp;
  addedImuMeasMsg.header.frame_id = slam_->frames_.imuFrame;
  addedImuMeasMsg.linear_acceleration.x = addedImuMeas(0);
  addedImuMeasMsg.linear_acceleration.y = addedImuMeas(1);
  addedImuMeasMsg.linear_acceleration.z = addedImuMeas(2);
  addedImuMeasMsg.angular_velocity.x = addedImuMeas(3);
  addedImuMeasMsg.angular_velocity.y = addedImuMeas(4);
  addedImuMeasMsg.angular_velocity.z = addedImuMeas(5);
  addedImuMeasPub_.publish(addedImuMeasMsg);
}

void OnlineRangeDataProcessorRos::poseStampedCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  if ((odometryCallBackEnabled_ || poseStampedWithCovarianceCallBackEnabled_)) {
    return;
  }

  poseStampedCallBackEnabled_ = true;

  geometry_msgs::Pose odomPose = msg->pose;
  processOdometryData(o3d_slam::getTransform(odomPose), fromRos(msg->header.stamp));
}

void OnlineRangeDataProcessorRos::poseStampedWithCovarianceCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
  // This is expected to be the default. So we dont return here.
  poseStampedWithCovarianceCallBackEnabled_ = true;

  geometry_msgs::Pose odomPose = msg->pose.pose;
  processOdometryData(o3d_slam::getTransform(odomPose), fromRos(msg->header.stamp));
  ROS_DEBUG_STREAM("Pose with covariance callback is called.");
}

void OnlineRangeDataProcessorRos::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  if ((poseStampedCallBackEnabled_ || poseStampedWithCovarianceCallBackEnabled_)) {
    // std::cout << "Already an odometry measurement for this timestamp. Skipping odometryCallback" << std::endl;
    return;
  }
  odometryCallBackEnabled_ = true;

  geometry_msgs::Pose odomPose;
  odomPose.orientation = msg->pose.pose.orientation;
  odomPose.position = msg->pose.pose.position;

  processOdometryData(o3d_slam::getTransform(odomPose), fromRos(msg->header.stamp));
}

void OnlineRangeDataProcessorRos::tryProcessPendingClouds() {
  std::lock_guard<std::mutex> lock(pendingCloudsMutex_);

  while (!pendingClouds_.empty()) {
    const auto& measurement = pendingClouds_.front();

    if (!slam_->isUseExistingMapEnabled() && slam_->isUsingOdometryTopic() && !slam_->isInitialTransformSet()) {
      return;
    }

    if (slam_->isUsingOdometryTopic()) {
      if (slam_->isOdometryPoseBufferEmpty()) {
        return;
      }

      if (slam_->isMeasurementOlderThanOdometryBuffer(measurement.timestamp_)) {
        ROS_WARN_STREAM("Dropping pending cloud older than the available odometry buffer. Stamp: " << measurement.timestamp_);
        slam_->discardCloudPipelineLatencyMeasurement(measurement.timestamp_);
        pendingClouds_.pop_front();
        continue;
      }

      if (!slam_->doesOdometryBufferBracketMeasurement(measurement.timestamp_)) {
        return;
      }
    }

    slam_->markCloudQueuedForProcessing(measurement.timestamp_, std::chrono::steady_clock::now());
    if (!slam_->addRangeScan(measurement.cloud_, measurement.timestamp_)) {
      ROS_WARN_STREAM("Failed to add a ready pending range scan. Dropping the measurement at stamp: " << measurement.timestamp_);
      slam_->discardCloudPipelineLatencyMeasurement(measurement.timestamp_);
      pendingClouds_.pop_front();
      continue;
    }

    pendingClouds_.pop_front();
  }
}

void OnlineRangeDataProcessorRos::publishCompletedMappingResultIfAvailable() {
  std::lock_guard<std::mutex> lock(completedMappingResultMutex_);

  const std::tuple<PointCloud, Time, Transform> cloudTimePair = slam_->getLatestRegisteredCloudTimestampPair();
  const std::tuple<Time, Transform> bestGuessTimePair = slam_->getLatestRegistrationBestGuess();

  const Time mappedTime = std::get<1>(cloudTimePair);
  const Time bestGuessTime = std::get<0>(bestGuessTimePair);
  if (!isTimeValid(mappedTime) || !isTimeValid(bestGuessTime)) {
    return;
  }

  if (std::get<0>(cloudTimePair).IsEmpty()) {
    return;
  }

  if (mappedTime != bestGuessTime) {
    return;
  }

  if (mappedTime == lastPublishedMappedResultTimestamp_) {
    return;
  }

  o3d_slam::publishCloud(std::get<0>(cloudTimePair), slam_->frames_.rangeSensorFrame, toRos(mappedTime), registeredCloudPub_);

  if (surfaceNormalPub_.getNumSubscribers() > 0u || surfaceNormalPub_.isLatched()) {
    auto surfaceNormalLineMarker{
        generateMarkersForSurfaceNormalVectors(std::get<0>(cloudTimePair), toRos(mappedTime), colorMap_[ColorKey::kRed])};

    if (surfaceNormalLineMarker != std::nullopt) {
      surfaceNormalPub_.publish(surfaceNormalLineMarker.value());
    }
  }

  lastPublishedMappedResultTimestamp_ = mappedTime;
}

}  // namespace o3d_slam
