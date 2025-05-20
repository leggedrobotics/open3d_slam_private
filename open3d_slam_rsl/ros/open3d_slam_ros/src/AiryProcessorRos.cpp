#include "open3d_slam_ros/AiryProcessorRos.hpp"
#include <algorithm>
#include <atomic>
#include <iostream>

using std::chrono::steady_clock;

namespace airy_processor {

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
  nh_->param<std::string>("cloud_topic", cloud_topic_, "/cloud");
  nh_->param<std::string>("pose_topic", pose_topic_, "/pose");

  std::cout << "[AiryProcessor] cloud_topic param: " << cloud_topic_ << '\n';
  std::cout << "[AiryProcessor] pose_topic param: " << pose_topic_ << '\n';

  processedPub_ = nh_->advertise<sensor_msgs::PointCloud2>("my_deskewed_cloud", 1, false);
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
  const auto t0 = steady_clock::now();

  auto deskewed = deskewPointCloud(msg);
  processedPub_.publish(deskewed);

  const auto t1 = steady_clock::now();
  const double ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
  ROS_DEBUG_STREAM_THROTTLE(1.0, "[AiryProcessor] deskew " << msg->width * msg->height << " pts in " << ms << " ms");
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

sensor_msgs::PointCloud2 AiryProcessorRos::deskewPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
  sensor_msgs::PointCloud2 ros_msg = *msg;

  // // Optional debug: print first 10 timestamps
  // try {
  //   bool found = false;
  //   int datatype = -1;
  //   for (const auto& f : ros_msg.fields) {
  //     if (f.name == "timestamp" || f.name == "time") {
  //       found = true;
  //       datatype = f.datatype;
  //       break;
  //     }
  //   }
  //   if (!found) throw std::runtime_error("timestamp field not found in PointCloud2");

  //   std::cout << "[DEBUG] First 10 point timestamps:\n";
  //   std::cout << std::setprecision(16);

  //   if (datatype == sensor_msgs::PointField::FLOAT64) {
  //     sensor_msgs::PointCloud2Iterator<double> iter_t(ros_msg, "timestamp");
  //     for (size_t i = 0; i < 10 && iter_t != iter_t.end(); ++i, ++iter_t) {
  //       std::cout << "Point " << i << " timestamp: " << *iter_t << "\n";
  //     }
  //   } else if (datatype == sensor_msgs::PointField::FLOAT32) {
  //     sensor_msgs::PointCloud2Iterator<float> iter_t(ros_msg, "timestamp");
  //     for (size_t i = 0; i < 10 && iter_t != iter_t.end(); ++i, ++iter_t) {
  //       std::cout << "Point " << i << " timestamp: " << *iter_t << "\n";
  //     }
  //   } else {
  //     throw std::runtime_error("timestamp field is not float or double");
  //   }
  // } catch (const std::exception& ex) {
  //   ROS_ERROR_STREAM("Error reading timestamps: " << ex.what());
  // }

  // ── validate fields once ──────────────────────────────────────────────────
  const FieldOffsets off = computeFieldOffsets(ros_msg);
  if (off.x < 0 || off.y < 0 || off.z < 0 || off.t < 0) {
    ROS_WARN_STREAM_THROTTLE(1.0, "Deskew: required fields missing");
    return ros_msg;
  }

  const size_t n_pts = ros_msg.width * ros_msg.height;
  const size_t step = ros_msg.point_step;
  uint8_t* base = ros_msg.data.data();

  Eigen::Isometry3d T_odom_base_ref;
  auto snapshot = pose_buffer_.snapshot();
  if (!PoseBuffer::query(snapshot, msg->header.stamp, T_odom_base_ref)) {
    ROS_WARN_STREAM_THROTTLE(1.0, "No reference pose for deskewing");
    return ros_msg;
  }

  const Eigen::Isometry3d T_odom_rslidar_ref = T_odom_base_ref * T_base_rslidar_;

  /* inverse split into rotation & translation for cheap per-point use */
  const Eigen::Matrix3d R_ref_inv = T_odom_rslidar_ref.linear().transpose();
  const Eigen::Vector3d t_ref_inv = -R_ref_inv * T_odom_rslidar_ref.translation();

#pragma omp parallel
  {
    unsigned local_skipped = 0;

#pragma omp for schedule(static, 2048)
    for (std::int64_t i = 0; i < static_cast<std::int64_t>(n_pts); ++i) {
      uint8_t* ptr = base + i * step;
      float* px = reinterpret_cast<float*>(ptr + off.x);
      float* py = reinterpret_cast<float*>(ptr + off.y);
      float* pz = reinterpret_cast<float*>(ptr + off.z);

      const double t_val = *reinterpret_cast<const double*>(ptr + off.t);
      ros::Time pt_time(t_val);

      /* trajectory lookup -------------------------------------------------- */
      Eigen::Isometry3d T_odom_base_pt;
      if (!PoseBuffer::query(snapshot, pt_time, T_odom_base_pt)) {
        ++local_skipped;
        continue;
      }

      /* build R_pt, t_pt without forming a 4×4 matrix ---------------------- */
      const Eigen::Matrix3d R_pt = T_odom_base_pt.linear() * T_base_rslidar_.linear();

      const Eigen::Vector3d t_pt = T_odom_base_pt.linear() * T_base_rslidar_.translation() + T_odom_base_pt.translation();

      /* p_ref = T_ref_inv * (R_pt * p_rsl + t_pt) -------------------------- */
      const Eigen::Vector3d p_rsl(*px, *py, *pz);
      const Eigen::Vector3d p_tmp = R_pt * p_rsl + t_pt;
      const Eigen::Vector3d p_ref = R_ref_inv * p_tmp + t_ref_inv;

      *px = static_cast<float>(p_ref.x());
      *py = static_cast<float>(p_ref.y());
      *pz = static_cast<float>(p_ref.z());
    }  // for

  }  // parallel region

  ros_msg.header.stamp = msg->header.stamp;
  return ros_msg;
}

}  // namespace airy_processor
