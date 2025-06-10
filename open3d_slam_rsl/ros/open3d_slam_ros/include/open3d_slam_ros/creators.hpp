/*
 * creators.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */
#include "open3d_slam_ros/OnlineRangeDataProcessorRos.hpp"
#include "open3d_slam_ros/RosbagRangeDataProcessorRos.hpp"

namespace o3d_slam {

std::shared_ptr<OnlineRangeDataProcessorRos> createOnlineDataProcessor(rclcpp::Node::SharedPtr nh);
std::shared_ptr<RosbagRangeDataProcessorRos> createRosbagDataProcessor(rclcpp::Node::SharedPtr nh);

std::shared_ptr<DataProcessorRos> dataProcessorFactory(rclcpp::Node::SharedPtr nh, bool isProcessAsFastAsPossible);

} /* namespace o3d_slam */
