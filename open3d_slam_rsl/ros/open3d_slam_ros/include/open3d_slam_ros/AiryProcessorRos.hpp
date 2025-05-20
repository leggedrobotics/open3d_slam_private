#pragma once
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <deque>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace airy_processor {

enum class ColorKey : uint8_t { kWhite = 0, kRed, kGreen, kBlue, kCyan, kYellow, kGold, kGrey, kLavender, kOrange, kBlack };

struct RgbaColorMap {
  using Values = std::vector<float>;
  const std::unordered_map<ColorKey, Values> rgb_ = {{ColorKey::kWhite, {1, 1, 1, 1}},
                                                     {ColorKey::kBlue, {0, 0, 1, 1}},
                                                     {ColorKey::kCyan, {0, 1, 1, 1}},
                                                     {ColorKey::kRed, {1, 0, 0, 1}},
                                                     {ColorKey::kGreen, {0, 1, 0, 1}},
                                                     {ColorKey::kGrey, {0.705, 0.674, 0.678, 1}},
                                                     {ColorKey::kLavender, {0.560, 0.501, 0.674, 1}},
                                                     {ColorKey::kYellow, {1, 1, 0.2, 1}},
                                                     {ColorKey::kGold, {0.898, 0.784, 0.462, 1}},
                                                     {ColorKey::kOrange, {1, 0.501, 0, 1}},
                                                     {ColorKey::kBlack, {0, 0, 0, 1}}};
  Values operator[](ColorKey id) const { return rgb_.at(id); }
};

// ───────────────────────────────── pose ring-buffer ─────────────────────────────
struct TimedPose {
  ros::Time stamp;
  Eigen::Isometry3d pose;
};

class PoseBuffer {
 public:
  void add(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  std::vector<TimedPose> snapshot() const;

  static bool query(const std::vector<TimedPose>& buf, const ros::Time& stamp, Eigen::Isometry3d& out_pose);

 private:
  std::deque<TimedPose> buffer_;
  mutable std::shared_mutex mtx_;
  const size_t max_size_ = 2000;
};

// ─────────────────────────────── ROS wrapper class ─────────────────────────────
class AiryProcessorRos {
 public:
  explicit AiryProcessorRos(const ros::NodeHandlePtr& nh);
  ~AiryProcessorRos() = default;

  void initCommonRosStuff();
  void subscribeCloud();

 protected:
  sensor_msgs::PointCloud2 deskewPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  struct FieldOffsets {
    int x = -1, y = -1, z = -1, t = -1;
  };
  FieldOffsets computeFieldOffsets(const sensor_msgs::PointCloud2& msg) const;

  ros::Publisher processedPub_;
  ros::Subscriber airySub_;
  ros::Subscriber pose_sub_;
  ros::NodeHandlePtr nh_;

  std::string cloud_topic_;
  std::string pose_topic_;

  PoseBuffer pose_buffer_;
  Eigen::Isometry3d T_base_rslidar_ = Eigen::Isometry3d::Identity();

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace airy_processor
