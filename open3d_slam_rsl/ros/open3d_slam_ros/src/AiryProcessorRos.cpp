#include "open3d_slam_ros/AiryProcessorRos.hpp"
#include <algorithm>
#include <atomic>
#include <iostream>
#include <numeric>
#include <vector>

using std::chrono::steady_clock;

namespace airy_processor {

struct DeskewPoint {
  size_t idx;
  float* px;
  float* py;
  float* pz;
  double timestamp;
  uint8_t* src_ptr;
};

void PoseBuffer::add(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
  Eigen::Isometry3d pose = Eigen::Translation3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) *
                           Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                              msg->pose.pose.orientation.z);

  {
    std::unique_lock lock(mtx_);
    buffer_.push_back({msg->header.stamp, pose});
    if (buffer_.size() > max_size_) buffer_.pop_front();
  }
}

std::vector<TimedPose> PoseBuffer::snapshot() const {
  std::shared_lock lock(mtx_);
  return {buffer_.begin(), buffer_.end()};
}

bool PoseBuffer::query(const std::vector<TimedPose>& buf, const ros::Time& stamp, Eigen::Isometry3d& out) {
  if (buf.size() < 2) return false;

  auto it = std::lower_bound(buf.begin(), buf.end(), stamp, [](const TimedPose& p, const ros::Time& t) { return p.stamp < t; });

  const TimedPose* p1;
  const TimedPose* p2;
  double tau;

  if (it == buf.begin()) {
    p1 = &buf[0];
    p2 = &buf[1];
  } else if (it == buf.end()) {
    p1 = &buf[buf.size() - 2];
    p2 = &buf.back();
  } else {
    p2 = &*it;
    p1 = &*(it - 1);
  }

  const double t1 = p1->stamp.toSec();
  const double t2 = p2->stamp.toSec();
  if (t1 == t2) return false;

  tau = (stamp.toSec() - t1) / (t2 - t1);

  Eigen::Quaterniond q1(p1->pose.rotation());
  Eigen::Quaterniond q2(p2->pose.rotation());
  Eigen::Quaterniond q = q1.slerp(tau, q2);
  Eigen::Vector3d tr = (1.0 - tau) * p1->pose.translation() + tau * p2->pose.translation();

  out = Eigen::Isometry3d::Identity();
  out.linear() = q.toRotationMatrix();
  out.translation() = tr;
  return true;
}

AiryProcessorRos::AiryProcessorRos(const ros::NodeHandlePtr& nh) : nh_(nh), tf_listener_(tf_buffer_) {}

// ------------------------------------------------------------------------------
void AiryProcessorRos::initCommonRosStuff() {
  nh_->param<std::string>("cloud_topic", cloud_topic_, "/rslidar_points");
  nh_->param<std::string>("pose_topic", pose_topic_, "/state_estimator/pose_in_odom");
  nh_->param<float>("distance_cutoff", distance_cutoff_, 0.5);

  std::cout << "[AiryProcessor] cloud_topic param: " << cloud_topic_ << '\n';
  std::cout << "[AiryProcessor] pose_topic param: " << pose_topic_ << '\n';
  std::cout << "[AiryProcessor] distance_cutoff param: " << distance_cutoff_ << '\n';

  processedPub_ = nh_->advertise<sensor_msgs::PointCloud2>("airy_deskewed_cloud", 1, false);
}

// ------------------------------------------------------------------------------
void AiryProcessorRos::subscribeCloud() {
  constexpr double TF_TIMEOUT = 1.0;
  constexpr double WAIT_INTERVAL = 0.5;
  constexpr double WARN_INTERVAL = 5.0;

  ros::Time start_time = ros::Time::now();
  ros::Duration warn_duration(WARN_INTERVAL);

  while (ros::ok()) {
    bool ready = tf_buffer_.canTransform("base", "rslidar", ros::Time(0), ros::Duration(TF_TIMEOUT));
    if (ready) {
      std::cout << "\033[1;32m[AiryProcessorRos] Found static TF base↔rslidar.\033[0m" << std::endl;
      break;
    }

    ros::Duration(WAIT_INTERVAL).sleep();

    ros::Time now = ros::Time::now();
    if ((now - start_time) >= warn_duration) {
      std::cout << "\033[1;33m[AiryProcessorRos] Waiting for static TF base↔rslidar...\033[0m" << std::endl;
      start_time = now;
    }
  }

  try {
    auto tf_msg = tf_buffer_.lookupTransform("base", "rslidar", ros::Time(0), ros::Duration(TF_TIMEOUT));
    Eigen::Quaterniond q(tf_msg.transform.rotation.w, tf_msg.transform.rotation.x, tf_msg.transform.rotation.y,
                         tf_msg.transform.rotation.z);
    T_base_rslidar_.linear() = q.toRotationMatrix();
    T_base_rslidar_.translation() =
        Eigen::Vector3d(tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z);

    isom3d_to_mat33_vec3(T_base_rslidar_, R_base_lidar_, t_base_lidar_);

  } catch (tf2::TransformException& ex) {
    throw std::runtime_error(std::string("Cannot start: base↔rslidar TF missing: ") + ex.what());
  }

  airySub_ = nh_->subscribe(cloud_topic_, 1, &AiryProcessorRos::cloudCallback, this);

  std::cout << "\033[1;32m[AiryProcessorRos] Subscribed to cloud topic successfully.\033[0m" << std::endl;

  pose_sub_ = nh_->subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      pose_topic_, 400, [this](const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) { pose_buffer_.add(msg); });

  std::cout << "\033[1;32m[AiryProcessorRos] Subscribed to pose topic successfully.\033[0m" << std::endl;
}

// ------------------------------------------------------------------------------
void AiryProcessorRos::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // const auto t0 = steady_clock::now();

  if (processedPub_.getNumSubscribers() == 0) {
    return;
  }
  auto deskewed = deskewPointCloud(msg);
  processedPub_.publish(deskewed);

  // const auto t1 = steady_clock::now();
  // const double ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
  // ROS_INFO_STREAM_THROTTLE(1.0, "[AiryProcessor] deskew " << msg->width * msg->height << " pts in " << ms << " ms");
}

AiryProcessorRos::FieldOffsets AiryProcessorRos::computeFieldOffsets(const sensor_msgs::PointCloud2& msg) const {
  FieldOffsets o;
  for (const auto& f : msg.fields) {
    if (f.name == "x")
      o.x = f.offset;
    else if (f.name == "y")
      o.y = f.offset;
    else if (f.name == "z")
      o.z = f.offset;
    else if (f.name == "timestamp" || f.name == "time")
      o.t = f.offset;
  }
  return o;
}

void AiryProcessorRos::isom3d_to_mat33_vec3(const Eigen::Isometry3d& iso, double R[3][3], double t[3]) {
  auto m = iso.linear();
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) R[r][c] = m(r, c);
  auto v = iso.translation();
  t[0] = v.x();
  t[1] = v.y();
  t[2] = v.z();
}

sensor_msgs::PointCloud2 AiryProcessorRos::deskewPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
  using clock = std::chrono::steady_clock;
  // auto t_block0 = clock::now();

  sensor_msgs::PointCloud2 ros_msg = *msg;

  const FieldOffsets off = computeFieldOffsets(ros_msg);
  if (off.x < 0 || off.y < 0 || off.z < 0 || off.t < 0) {
    ROS_WARN_STREAM_THROTTLE(1.0, "Deskew: required fields missing");
    return ros_msg;
  }

  const size_t n_pts = ros_msg.width * ros_msg.height;
  const size_t step = ros_msg.point_step;
  uint8_t* base = ros_msg.data.data();

  // Gather validity mask and timestamps
  // auto t_block1 = clock::now();
  std::vector<uint8_t> valid(n_pts, 0);
  std::vector<double> timestamps(n_pts, 0.0);

  int timestamp_field_type = -1;
  for (const auto& f : ros_msg.fields) {
    if (f.offset == off.t) {
      timestamp_field_type = f.datatype;
      break;
    }
  }
  if (timestamp_field_type != sensor_msgs::PointField::FLOAT64 && timestamp_field_type != sensor_msgs::PointField::FLOAT32) {
    ROS_WARN_STREAM_THROTTLE(1.0, "Deskew: timestamp field not FLOAT64 or FLOAT32");
    return ros_msg;
  }

#pragma omp parallel for schedule(static, 4096)
  for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(n_pts); ++i) {
    uint8_t* ptr = base + i * step;
    float* pz = reinterpret_cast<float*>(ptr + off.z);
    float range = *pz;
    if (range < distance_cutoff_) continue;

    double t_val;
    if (timestamp_field_type == sensor_msgs::PointField::FLOAT64)
      t_val = *reinterpret_cast<const double*>(ptr + off.t);
    else
      t_val = static_cast<double>(*reinterpret_cast<const float*>(ptr + off.t));

    valid[i] = 1;
    timestamps[i] = t_val;
  }
  // auto t_block2 = clock::now();

  // Prefix sum: output indices for kept points
  std::vector<size_t> output_idx(n_pts, 0);
  size_t n_kept = 0;
  for (size_t i = 0; i < n_pts; ++i) {
    output_idx[i] = n_kept;
    n_kept += valid[i];
  }
  if (n_kept == 0) {
    ros_msg.data.clear();
    ros_msg.width = 0;
    ros_msg.height = 1;
    ros_msg.row_step = 0;
    return ros_msg;
  }
  // auto t_block3 = clock::now();

  // Batch interpolate unique poses for all kept timestamps
  std::vector<TimedPose> pose_snap = pose_buffer_.snapshot();

  // Gather unique timestamps (sorted)
  std::vector<double> all_times;
  all_times.reserve(n_kept);
  for (size_t i = 0; i < n_pts; ++i)
    if (valid[i]) all_times.push_back(timestamps[i]);
  std::sort(all_times.begin(), all_times.end());
  auto last = std::unique(all_times.begin(), all_times.end());
  all_times.erase(last, all_times.end());
  size_t n_times = all_times.size();

  std::vector<Eigen::Isometry3d> pose_results(n_times);

#pragma omp parallel for schedule(static, 1024)
  for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(n_times); ++i) {
    double t = all_times[i];
    ros::Time stamp(t);

    auto it =
        std::lower_bound(pose_snap.begin(), pose_snap.end(), stamp, [](const TimedPose& p, const ros::Time& t) { return p.stamp < t; });

    const TimedPose* p1;
    const TimedPose* p2;
    if (it == pose_snap.begin()) {
      p1 = &pose_snap[0];
      p2 = &pose_snap[1];
    } else if (it == pose_snap.end()) {
      p1 = &pose_snap[pose_snap.size() - 2];
      p2 = &pose_snap.back();
    } else {
      p2 = &*it;
      p1 = &*(it - 1);
    }

    double t1 = p1->stamp.toSec();
    double t2 = p2->stamp.toSec();
    double tau = (t2 != t1) ? ((t - t1) / (t2 - t1)) : 0.0;

    Eigen::Quaterniond q1(p1->pose.rotation());
    Eigen::Quaterniond q2(p2->pose.rotation());
    Eigen::Quaterniond q = q1.slerp(tau, q2);
    Eigen::Vector3d tr = (1.0 - tau) * p1->pose.translation() + tau * p2->pose.translation();

    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = q.toRotationMatrix();
    result.translation() = tr;
    pose_results[i] = result;
  }
  // auto t_block4 = clock::now();

  // Precompute pose_idx for each valid point
  std::vector<size_t> pose_idx(n_pts, 0);
#pragma omp parallel for schedule(static, 4096)
  for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(n_pts); ++i) {
    if (!valid[i]) continue;
    double t = timestamps[i];
    size_t idx = std::lower_bound(all_times.begin(), all_times.end(), t) - all_times.begin();
    pose_idx[i] = idx;
  }
  // auto t_block5 = clock::now();

  // Prepare reference pose and Lidar-to-base transform at the scan time
  Eigen::Isometry3d T_odom_base_ref;
  if (!PoseBuffer::query(pose_snap, msg->header.stamp, T_odom_base_ref)) {
    ROS_WARN_STREAM_THROTTLE(1.0, "No reference pose for deskewing");
    return ros_msg;
  }

  double R_base_ref[3][3], t_base_ref[3];
  isom3d_to_mat33_vec3(T_odom_base_ref, R_base_ref, t_base_ref);
  double R_odom_rslidar_ref[3][3], t_odom_rslidar_ref[3];
  mat3x3_mult(R_base_ref, R_base_lidar_, R_odom_rslidar_ref);
  double tmp[3];
  mat3x3_vec3_mult(R_base_ref, t_base_lidar_, tmp);
  t_odom_rslidar_ref[0] = tmp[0] + t_base_ref[0];
  t_odom_rslidar_ref[1] = tmp[1] + t_base_ref[1];
  t_odom_rslidar_ref[2] = tmp[2] + t_base_ref[2];

  double R_ref_inv[3][3];
  mat3x3_transpose(R_odom_rslidar_ref, R_ref_inv);
  double t_ref_inv[3];
  mat3x3_vec3_mult(R_ref_inv, t_odom_rslidar_ref, tmp);
  t_ref_inv[0] = -tmp[0];
  t_ref_inv[1] = -tmp[1];
  t_ref_inv[2] = -tmp[2];
  // auto t_block6 = clock::now();

  // Allocate output buffer for kept points
  std::vector<uint8_t> compacted(n_kept * step);

#pragma omp parallel for schedule(static, 2048)
  for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(n_pts); ++i) {
    if (!valid[i]) continue;
    uint8_t* ptr = base + i * step;
    float* px = reinterpret_cast<float*>(ptr + off.x);
    float* py = reinterpret_cast<float*>(ptr + off.y);
    float* pz = reinterpret_cast<float*>(ptr + off.z);

    size_t idx = pose_idx[i];
    const Eigen::Isometry3d& pose = pose_results[idx];

    // Compose full LiDAR pose at this time
    double R_base_pt[3][3], t_base_pt[3];
    isom3d_to_mat33_vec3(pose, R_base_pt, t_base_pt);

    double R_pt[3][3];
    mat3x3_mult(R_base_pt, R_base_lidar_, R_pt);

    mat3x3_vec3_mult(R_base_pt, t_base_lidar_, tmp);
    double t_pt[3] = {tmp[0] + t_base_pt[0], tmp[1] + t_base_pt[1], tmp[2] + t_base_pt[2]};

    float p_rsl_f[3] = {*px, *py, *pz};

    double p_tmp[3];
    mat3x3_vec3_mult(R_pt, p_rsl_f, p_tmp);
    p_tmp[0] += t_pt[0];
    p_tmp[1] += t_pt[1];
    p_tmp[2] += t_pt[2];

    double p_ref[3];
    mat3x3_vec3_mult(R_ref_inv, p_tmp, p_ref);
    p_ref[0] += t_ref_inv[0];
    p_ref[1] += t_ref_inv[1];
    p_ref[2] += t_ref_inv[2];

    // Write back transformed coordinates
    size_t out_idx = output_idx[i];
    uint8_t* dst = compacted.data() + out_idx * step;
    std::memcpy(dst, ptr, step);
    float* out_px = reinterpret_cast<float*>(dst + off.x);
    float* out_py = reinterpret_cast<float*>(dst + off.y);
    float* out_pz = reinterpret_cast<float*>(dst + off.z);
    *out_px = static_cast<float>(p_ref[0]);
    *out_py = static_cast<float>(p_ref[1]);
    *out_pz = static_cast<float>(p_ref[2]);
  }
  // auto t_block7 = clock::now();

  ros_msg.data = std::move(compacted);
  ros_msg.width = n_kept;
  ros_msg.height = 1;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.header.stamp = msg->header.stamp;

  // auto ms = [](const auto& t0, const auto& t1) { return std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
  // };

  // ROS_INFO_STREAM("[Profile] total:         " << ms(t_block0, t_block7) << " ms");
  // ROS_INFO_STREAM("[Profile] valid+ts:      " << ms(t_block1, t_block2) << " ms");
  // ROS_INFO_STREAM("[Profile] prefix_sum:    " << ms(t_block2, t_block3) << " ms");
  // ROS_INFO_STREAM("[Profile] pose interp:   " << ms(t_block3, t_block4) << " ms");
  // ROS_INFO_STREAM("[Profile] pose_idx:      " << ms(t_block4, t_block5) << " ms");
  // ROS_INFO_STREAM("[Profile] ref_pose:      " << ms(t_block5, t_block6) << " ms");
  // ROS_INFO_STREAM("[Profile] deskew+copy:   " << ms(t_block6, t_block7) << " ms");

  return ros_msg;
}
}  // namespace airy_processor
