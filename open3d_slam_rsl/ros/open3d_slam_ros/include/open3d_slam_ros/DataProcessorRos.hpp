/*
 * DataProcessorRos.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once
#include <ros/ros.h>

#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/typedefs.hpp"

namespace o3d_slam {

class DataProcessorRos {
 public:
  DataProcessorRos(ros::NodeHandlePtr nh);
  virtual ~DataProcessorRos() = default;

  virtual void initialize() = 0;
  virtual void startProcessing() = 0;
  virtual void processMeasurement(const PointCloud& cloud, const Time& timestamp);
  virtual void processOdometry(const Transform& cloud, const Time& timestamp);
  void accumulateAndProcessRangeData(const PointCloud& cloud, const Time& timestamp);
  void processOdometryData(const Transform& transform, const Time& timestamp);
  void initCommonRosStuff();
  std::shared_ptr<SlamWrapper> getSlamPtr();

 protected:
  size_t numAccumulatedRangeDataCount_ = 0;
  size_t numPointCloudsReceived_ = 0;
  size_t numAccumulatedRangeDataDesired_ = 1;
  PointCloud accumulatedCloud_;

  ros::Publisher rawCloudPub_;
  ros::Publisher registeredCloudPub_;
  ros::Publisher offlinePathPub_;
  ros::Publisher offlineDifferenceLinePub_;
  ros::Publisher offlineBestGuessPathPub_;
  ros::Publisher addedImuMeasPub_;
  std::string cloudTopic_{""};
  std::string odometryTopic_{""};
  std::string poseStampedWithCovarianceTopic_{""};
  std::string poseStampedTopic_{""};

  std::shared_ptr<SlamWrapper> slam_;
  ros::NodeHandlePtr nh_;
};

}  // namespace o3d_slam
