/*
 * helpers_ros.cpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#include "open3d_slam_ros/helpers_ros.hpp"
#include <random>
#include "open3d_slam/SubmapCollection.hpp"
// ROS 2 stuff
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include "open3d_conversions/open3d_conversions.hpp"
#include "open3d_slam_ros/Color.hpp"

namespace o3d_slam {

void publishSubmapCoordinateAxes(const SubmapCollection& submaps, const std::string& frame_id, const rclcpp::Time& timestamp,
                                 const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& pub) {
  visualization_msgs::msg::MarkerArray msg;
  int id = 0;
  msg.markers.reserve(2 * submaps.getNumSubmaps());
  for (size_t j = 0; j < submaps.getNumSubmaps(); ++j) {
    const Submap& submap = submaps.getSubmap(j);
    visualization_msgs::msg::Marker axes, text;
    drawAxes(submap.getMapToSubmapCenter(), Eigen::Quaterniond(submap.getMapToSubmapOrigin().rotation()), 0.8, 0.08, &axes);
    axes.ns = "submap_" + std::to_string(j);
    axes.header.frame_id = frame_id;
    axes.header.stamp = timestamp;
    axes.id = submap.getId();
    msg.markers.push_back(axes);
    text = axes;
    text.pose.position.x += 0.3;
    text.pose.position.y += 0.3;
    text.scale.x = text.scale.y = text.scale.z = 0.4;
    text.color.r = text.color.g = text.color.b = 1.0;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.text = "(" + std::to_string(j) + ")";
    text.ns = "submap_id_" + std::to_string(j);
    text.id = id++;
    msg.markers.push_back(text);
  }
  pub->publish(msg);
}

void assembleColoredPointCloud(const SubmapCollection& submaps, open3d::geometry::PointCloud* cloud) {
  if (submaps.isEmpty()) {
    return;
  }
  std::mt19937 rndGen;
  rndGen.seed(time(NULL));
  const int nPoints = submaps.getTotalNumPoints();
  cloud->points_.reserve(nPoints);

  if (cloud->HasColors()) {
    cloud->colors_.reserve(nPoints);
    for (size_t j = 0; j < submaps.getNumSubmaps(); ++j) {
      const Submap& submap = submaps.getSubmap(j);
      const auto color = Color::getColor(j % (Color::numColors_ - 2) + 2);
      const PointCloud map = submap.getMapPointCloudCopy();
      for (size_t i = 0; i < map.points_.size(); ++i) {
        cloud->points_.push_back(map.points_.at(i));
        cloud->colors_.emplace_back(Eigen::Vector3d(color.msg_.r, color.msg_.g, color.msg_.b));

      }
    }

  } else {
    std::uniform_int_distribution<int> rndInt(2, 12);
    for (size_t j = 0; j < submaps.getNumSubmaps(); ++j) {
      const Submap& submap = submaps.getSubmap(j);
      const PointCloud map = submap.getMapPointCloudCopy();
      for (size_t i = 0; i < map.points_.size(); ++i) {
        cloud->points_.push_back(map.points_.at(i));
      }
    }
  }
}

void publishCloud(const open3d::geometry::PointCloud& cloud, const std::string& frame_id, const rclcpp::Time& timestamp,
                  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub) {
  if (pub->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 msg;
    const PointCloud copy = cloud;
    open3d_conversions::open3dToRos(copy, msg, frame_id);
    msg.header.stamp = timestamp;
    pub->publish(msg);
  }
}

void publishTfTransform(const Eigen::Matrix4d& Mat, const rclcpp::Time& time,
                        const std::string& parentFrame, const std::string& childFrame,
                        tf2_ros::TransformBroadcaster* broadcaster) {
  broadcaster->sendTransform(toRos(Mat, time, parentFrame, childFrame));
 }


bool lookupTransform(const std::string& target_frame, const std::string& source_frame, const rclcpp::Time& time,
                     const tf2_ros::Buffer& tfBuffer, Eigen::Isometry3d& transform) {
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.05));
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(rclcpp::get_logger("open3d_slam_ros"), "caught exception while looking up the tf: %s", ex.what());
    transform = Eigen::Isometry3d::Identity();
    return false;
  }
  transform = tf2::transformToEigen(transformStamped.transform);
  return true;
}




geometry_msgs::msg::Pose getPose(const Eigen::MatrixXd& T) {
  geometry_msgs::msg::Pose pose;
  Eigen::Affine3d eigenTr;
  eigenTr.matrix() = T;
  pose = tf2::toMsg(eigenTr);
  return pose;
}

o3d_slam::Transform getTransform(const geometry_msgs::msg::Pose& pose) {
    // Create a Transform object from Pose
    geometry_msgs::msg::Transform transform_msg;
    transform_msg.translation.x = pose.position.x;
    transform_msg.translation.y = pose.position.y;
    transform_msg.translation.z = pose.position.z;
    transform_msg.rotation = pose.orientation;

    // Now use tf2::transformToEigen with the Transform message
    Transform transform = tf2::transformToEigen(transform_msg);
    return transform;
}


geometry_msgs::msg::TransformStamped toRos(const Eigen::Matrix4d& Mat, const rclcpp::Time& time, const std::string& frame,
                                           const std::string& childFrame) {
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = time;
  transformStamped.header.frame_id = frame;
  transformStamped.child_frame_id = childFrame;
  const auto pose = getPose(Mat);
  transformStamped.transform.translation.x = pose.position.x;
  transformStamped.transform.translation.y = pose.position.y;
  transformStamped.transform.translation.z = pose.position.z;
  transformStamped.transform.rotation = pose.orientation;
  return transformStamped;
}

geometry_msgs::msg::Point createPoint(double x, double y, double z) {
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

void drawAxes(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double scale, double line_width, visualization_msgs::msg::Marker* marker) {
  marker->colors.resize(6);
  marker->points.resize(6);
  marker->points[0] = createPoint(0, 0, 0);
  marker->points[1] = createPoint(1 * scale, 0, 0);
  marker->points[2] = createPoint(0, 0, 0);
  marker->points[3] = createPoint(0, 1 * scale, 0);
  marker->points[4] = createPoint(0, 0, 0);
  marker->points[5] = createPoint(0, 0, 1 * scale);

  marker->color = Color::Black().msg_;  // Assign the ColorRGBA object from Color to Marker
  marker->colors[0] = Color::Red().msg_;
  marker->colors[1] = Color::Red().msg_;
  marker->colors[2] = Color::Green().msg_;
  marker->colors[3] = Color::Green().msg_;
  marker->colors[4] = Color::Blue().msg_;
  marker->colors[5] = Color::Blue().msg_;

  marker->scale.x = line_width;  // rest is unused
  marker->type = visualization_msgs::msg::Marker::LINE_LIST;
  marker->action = visualization_msgs::msg::Marker::ADD;

  marker->pose.position.x = p.x();
  marker->pose.position.y = p.y();
  marker->pose.position.z = p.z();

  marker->pose.orientation.x = q.x();
  marker->pose.orientation.y = q.y();
  marker->pose.orientation.z = q.z();
  marker->pose.orientation.w = q.w();
}


rclcpp::Time toRos(Time time) {
  int64_t uts_timestamp = toUniversal(time);
  int64_t ns_since_unix_epoch = (uts_timestamp - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
  rclcpp::Time ros_time(ns_since_unix_epoch);
  if (ns_since_unix_epoch < 0) {
    std::cerr << "ERROR: nanoseconds since unix epoch is: " << ns_since_unix_epoch << " which is impossible!!!! \n";
    std::cerr << "       ROS time will throw you an exception fo sho!!!! \n";
    std::cerr << "       Are you playing the rosbag with --clock??? \n";
    std::cerr << "       If yes, did you set use_sim_time to true ??? \n";
    std::cout << "Universal time: " << uts_timestamp << std::endl;
  }
  return ros_time;
}

Time fromRos(const rclcpp::Time& time) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  int64_t sec = time.seconds();
  int64_t nsec = time.nanoseconds() % 1000000000ll;
  return fromUniversal((sec + kUtsEpochOffsetFromUnixEpochInSeconds) * 10000000ll +
                       (nsec + 50) / 100);  // + 50 to get the rounding correct.
}

} /* namespace o3d_slam */
