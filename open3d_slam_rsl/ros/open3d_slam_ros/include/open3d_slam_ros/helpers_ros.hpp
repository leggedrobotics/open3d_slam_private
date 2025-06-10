/*
 * helpers_ros.hpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#pragma once
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <open3d/geometry/MeshBase.h>
#include <open3d/geometry/PointCloud.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include "open3d_slam/Transform.hpp"
#include "open3d_slam/time.hpp"

namespace o3d_slam {

class SubmapCollection;

void publishSubmapCoordinateAxes(const SubmapCollection& submaps, const std::string& frame_id, const rclcpp::Time& timestamp,
                                 const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& pub);
geometry_msgs::msg::Point createPoint(double x, double y, double z);
void drawAxes(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double scale, double line_width, visualization_msgs::msg::Marker* marker);

void assembleColoredPointCloud(const SubmapCollection& submaps, open3d::geometry::PointCloud* cloud);

void publishCloud(const open3d::geometry::PointCloud& cloud, const std::string& frame_id, const rclcpp::Time& timestamp,
                  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub);

geometry_msgs::msg::Pose getPose(const Eigen::MatrixXd& T);

o3d_slam::Transform getTransform(const geometry_msgs::msg::Pose& pose);

geometry_msgs::msg::TransformStamped toRos(const Eigen::Matrix4d& Mat, const rclcpp::Time& time, const std::string& frame,
                                           const std::string& childFrame);

void publishTfTransform(const Eigen::Matrix4d& T, const rclcpp::Time& timestamp, const std::string& parentFrame, const std::string& childFrame, tf2_ros::TransformBroadcaster* broadcaster);


bool lookupTransform(const std::string& target_frame, const std::string& source_frame, const rclcpp::Time& time,
                     const tf2_ros::Buffer& tfBuffer, Eigen::Isometry3d& transform);

rclcpp::Time toRos(Time time);

Time fromRos(const rclcpp::Time& time);

template <typename Msg>
void publishIfSubscriberExists(const Msg& msg, const typename rclcpp::Publisher<Msg>::SharedPtr& pub) {
  if (pub->get_subscription_count() > 0) {
    pub->publish(msg);
  }
}

// Repeated functionality from GraphMsfRos by Julian Nubert. Subject to its license.
template <typename T>
inline void printKey(const std::string& key, T value) {
  std::cout << "\033[92m"
            << "Open3d SLAM "
            << "\033[0m" << key << "  set to: " << value << std::endl;
}

template <>
inline void printKey(const std::string& key, std::vector<double> vector) {
  std::cout << "\033[92m"
            << "Open3d SLAM "
            << "\033[0m" << key << " set to: ";
  for (const auto& element : vector) {
    std::cout << element << ",";
  }
  std::cout << std::endl;
}

// Parameter retrieval for ROS 2 Node
template <typename T>
T tryGetParam(const std::string& key, const rclcpp::Node& node) {
  T value;
  if (node.get_parameter(key, value)) {
    printKey(key, value);
    return value;
  } else if (node.get_parameter("/" + key, value)) {
    printKey("/" + key, value);
    return value;
  }
  throw std::runtime_error("Open3d SLAM - " + key + " not specified.");
}

} /* namespace o3d_slam */
