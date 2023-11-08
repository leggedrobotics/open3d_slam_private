/*
 * OnlineDataProcessorRos.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam_ros/DataProcessorRos.hpp"
#include "open3d_slam_ros/ImuBuffer.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace o3d_slam {

class OnlineRangeDataProcessorRos : public DataProcessorRos {
  using BASE = DataProcessorRos;

 public:
  OnlineRangeDataProcessorRos(ros::NodeHandlePtr nh);
  ~OnlineRangeDataProcessorRos() override = default;

  void initialize() override;
  void startProcessing() override;
  void processMeasurement(const PointCloud& cloud, const Time& timestamp) override;
  void processOdometry(const Transform& cloud, const Time& timestamp) override;

  bool test_ = true;

 private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void poseStampedCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void poseStampedWithCovarianceCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_ptr);
  void publishAddedImuMeas_(const Eigen::Matrix<double, 6, 1>& addedImuMeas, const ros::Time& stamp);

  ros::Subscriber cloudSubscriber_;
  ros::Subscriber odometrySubscriber_;
  ros::Subscriber poseStampedCovarianceSubscriber_;
  ros::Subscriber poseStampedSubscriber_;
  ros::Subscriber imuSubscriber_;

  bool poseStampedCallBackEnabled_ = false;
  bool odometryCallBackEnabled_ = false;
  bool poseStampedWithCovarianceCallBackEnabled_ = false;
  bool isAttitudeInitialized_ = false;

  std::shared_ptr<ImuBuffer> imuBufferPtr_;
	//tf2_ros::Buffer tfBuffer_;
	//tf2_ros::TransformListener tfListener_;
  //std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

};

}  // namespace o3d_slam
