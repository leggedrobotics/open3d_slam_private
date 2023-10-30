/*
 * OnlineRangeDataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/OnlineRangeDataProcessorRos.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam/time.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"
namespace o3d_slam {

OnlineRangeDataProcessorRos::OnlineRangeDataProcessorRos(ros::NodeHandlePtr nh) : BASE(nh){
  //tfBroadcaster_.reset(new tf2_ros::TransformBroadcaster());
}

void OnlineRangeDataProcessorRos::initialize() {
  initCommonRosStuff();
  slam_ = std::make_shared<SlamWrapperRos>(nh_);
  imuBufferPtr_ = std::make_shared<ImuBuffer>();
  // If this is calling the ros wrapper version. Which overrides the base. The base function is later called within the overridden version.
  slam_->loadParametersAndInitialize();
}

void OnlineRangeDataProcessorRos::startProcessing() {
  slam_->startWorkers();
  cloudSubscriber_ = nh_->subscribe(cloudTopic_, 10, &OnlineRangeDataProcessorRos::cloudCallback, this, ros::TransportHints().tcpNoDelay());
  
  // Redundant listening of pose topics. It is really tiresome to the developer to keep the support for all types.
  poseStampedSubscriber_ = nh_->subscribe(poseStampedTopic_, 400, &OnlineRangeDataProcessorRos::poseStampedCallback, this, ros::TransportHints().tcpNoDelay());
  odometrySubscriber_ = nh_->subscribe(odometryTopic_, 400, &OnlineRangeDataProcessorRos::odometryCallback, this, ros::TransportHints().tcpNoDelay());
  poseStampedCovarianceSubscriber_ = nh_->subscribe(poseStampedWithCovarianceTopic_, 400, &OnlineRangeDataProcessorRos::poseStampedWithCovarianceCallback, this, ros::TransportHints().tcpNoDelay());
  
  // Currently harcoded to ANYmal, additional support will follow.
  imuSubscriber_ = nh_->subscribe<sensor_msgs::Imu>("/sensors/imu", 400, &OnlineRangeDataProcessorRos::imuCallback, this, ros::TransportHints().tcpNoDelay());

  // Number of spinners should be equal to the number of active subscribers
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  slam_->stopWorkers();
}

void OnlineRangeDataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {

  // Add the range scan to the pointcloud processing buffer. This is actually a buffer with size 1, so no queue.
  if (!slam_->addRangeScan(cloud, timestamp)){
    return;
  }

  // Re-publish the raw point cloud for visualization purposes.
  o3d_slam::publishCloud(cloud, slam_->frames_.rangeSensorFrame, toRos(timestamp), rawCloudPub_);
  
  // TODO(TT) Is this the best place to do this?
  // Get the latest registered point cloud and publish it.
  std::tuple<PointCloud, Time, Transform> cloudTimePair = slam_->getLatestRegisteredCloudTimestampPair();
  const bool isCloudEmpty = std::get<0>(cloudTimePair).IsEmpty();
  if (isTimeValid(std::get<1>(cloudTimePair)) && !isCloudEmpty) {
    o3d_slam::publishCloud(std::get<0>(cloudTimePair), slam_->frames_.rangeSensorFrame, toRos(std::get<1>(cloudTimePair)), registeredCloudPub_);
  }

  return;
}

void OnlineRangeDataProcessorRos::processOdometry(const Transform& transform, const Time& timestamp) {

  // When there is no IMU msg available we need to bypass this condition.
  if (!slam_->isIMUattitudeInitializationEnabled()){
    isAttitudeInitialized_ = true;
  }
  
  if (!isAttitudeInitialized_){
    ROS_WARN_STREAM_THROTTLE(0.2, "Attitude not initialized yet. Throttled 0.2s");
    return;
  }
  
  if (!slam_->addOdometryPoseToBuffer(transform, timestamp)){
    return;
  }

}

void OnlineRangeDataProcessorRos::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  open3d::geometry::PointCloud cloud;
  open3d_conversions::rosToOpen3d(msg, cloud, false);
  const Time timestamp = fromRos(msg->header.stamp);
  accumulateAndProcessRangeData(cloud, timestamp);
}

void OnlineRangeDataProcessorRos::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_ptr) {
  if (!slam_->isIMUattitudeInitializationEnabled())
  {
    isAttitudeInitialized_ = true;
    return;
  }
  
  // TODO(TT) Need to convert the calculated attitude to the sensor frame, I think...
  //Transform imuToLidar = Transform::Identity();
  //if(o3d_slam::lookupTransform("imu_link", "lidar", imu_ptr->header.stamp, tfBuffer_, imuToLidar)){
    //o3d_slam::publishTfTransform(imuToLidar.matrix(), ros::Time::now(), "imu_link_o3d", slam_->frames_.rangeSensorFrame, tfBroadcaster_.get());
  //}

  //Add to buffer
  Eigen::Vector3d linearAcc(imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imu_ptr->angular_velocity.x, imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z);

  Eigen::Matrix<double, 6, 1> addedImuMeasurements;
  addedImuMeasurements = imuBufferPtr_->addToImuBuffer(imu_ptr->header.stamp.toSec(), linearAcc, angularVel);
  publishAddedImuMeas_(addedImuMeasurements, imu_ptr->header.stamp);

  if (isAttitudeInitialized_){
    return;
  }
  
  Eigen::Quaterniond initAttitude = Eigen::Quaterniond::Identity();
  Eigen::Vector3d gyrBias = Eigen::Vector3d::Zero();
  double estimatedGravityMagnitude{0.0};
  if (!(imuBufferPtr_->estimateAttitudeFromImu(initAttitude, estimatedGravityMagnitude, gyrBias))){
    ROS_WARN_STREAM_THROTTLE(0.2, "Could not estimate attitude from IMU yet. Throttled 0.2s");
    return;
  }

  isAttitudeInitialized_ = true;
  Eigen::Vector3d gravityVector = Eigen::Vector3d(0, 0, 9.80665);
  Eigen::Vector3d estimatedGravityVector = Eigen::Vector3d(0, 0, estimatedGravityMagnitude);
  Eigen::Vector3d gravityVectorError = estimatedGravityVector - gravityVector;
  Eigen::Vector3d gravityVectorErrorInImuFrame = initAttitude.inverse().matrix() * gravityVectorError;
  std::cout << " Gravity error in IMU frame is: " << gravityVectorErrorInImuFrame.transpose()
        << std::endl;

  /*Eigen::Quaterniond validityCheck = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX());
            */

  Transform initialTransform = Transform::Identity();
  initialTransform.affine().matrix().block<3, 3>(0, 0) = initAttitude.matrix();//;.inverse(); // TODO, CHECK THE INVERSE?
  slam_->setInitialTransform(initialTransform.matrix());
}

void OnlineRangeDataProcessorRos::publishAddedImuMeas_(const Eigen::Matrix<double, 6, 1>& addedImuMeas, const ros::Time& stamp) {
  // Publish added imu measurement
  sensor_msgs::Imu addedImuMeasMsg;
  addedImuMeasMsg.header.stamp = stamp;
  addedImuMeasMsg.header.frame_id = "imu_link";
  addedImuMeasMsg.linear_acceleration.x = addedImuMeas(0);
  addedImuMeasMsg.linear_acceleration.y = addedImuMeas(1);
  addedImuMeasMsg.linear_acceleration.z = addedImuMeas(2);
  addedImuMeasMsg.angular_velocity.x = addedImuMeas(3);
  addedImuMeasMsg.angular_velocity.y = addedImuMeas(4);
  addedImuMeasMsg.angular_velocity.z = addedImuMeas(5);
  addedImuMeasPub_.publish(addedImuMeasMsg);
}

void OnlineRangeDataProcessorRos::poseStampedCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  
  if ((odometryCallBackEnabled_ || poseStampedWithCovarianceCallBackEnabled_)){
    return;
  }

  poseStampedCallBackEnabled_ = true;

  geometry_msgs::Pose odomPose = msg->pose;
  processOdometryData(o3d_slam::getTransform(odomPose), fromRos(msg->header.stamp));

}

void OnlineRangeDataProcessorRos::poseStampedWithCovarianceCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){

  if ((poseStampedCallBackEnabled_ || odometryCallBackEnabled_))
  {
    //std::cout << "Already an odometry measurement for this timestamp. Skipping poseStampedWithCovarianceCallback" << std::endl;
    return;
  }

  poseStampedWithCovarianceCallBackEnabled_ = true;

  geometry_msgs::Pose odomPose = msg->pose.pose;
  processOdometryData(o3d_slam::getTransform(odomPose), fromRos(msg->header.stamp));
}

void OnlineRangeDataProcessorRos::odometryCallback(const nav_msgs::OdometryConstPtr& msg){
  
  //
  if ((poseStampedCallBackEnabled_ || poseStampedWithCovarianceCallBackEnabled_))
  {
    //std::cout << "Already an odometry measurement for this timestamp. Skipping odometryCallback" << std::endl;
    return;
  }
  odometryCallBackEnabled_ = true;

  geometry_msgs::Pose odomPose;
  odomPose.orientation = msg->pose.pose.orientation;
  odomPose.position = msg->pose.pose.position;

  processOdometryData(o3d_slam::getTransform(odomPose), fromRos(msg->header.stamp));
}

}  // namespace o3d_slam
