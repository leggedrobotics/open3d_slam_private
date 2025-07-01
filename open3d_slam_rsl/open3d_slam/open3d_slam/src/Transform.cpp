/*
 * Transform.cpp
 *
 *  Created on: Nov 8, 2021
 *      Author: jelavice
 */

#include "open3d_slam/Transform.hpp"
#include <glog/logging.h>
#include <iostream>
#include <string>
#include "Eigen/Geometry"

namespace o3d_slam {

/* ------------------------------------------------------------------------- */
/*  Extrapolate *outside* the interval.                                      */
/*  If query < start.time_  : it linearly projects backwards.                 */
/*  If query > end.time_    : it linearly projects forwards.                  */
/*  Inside the interval it simply delegates to interpolate().               */
/* ------------------------------------------------------------------------- */
TimestampedTransform extrapolate(const TimestampedTransform& start, const TimestampedTransform& end, const Time& query) {
  /* swap to ensure start.time_ ≤ end.time_ for the math */
  const TimestampedTransform& s = (start.time_ <= end.time_) ? start : end;
  const TimestampedTransform& e = (start.time_ <= end.time_) ? end : start;

  /* inside the segment → ordinary interpolation */
  if (query >= s.time_ && query <= e.time_) return interpolate(s, e, query);

  /* identical stamps → cannot extrapolate velocity */
  constexpr double kEps = 1e-12;
  const double duration = toSeconds(e.time_ - s.time_);
  if (duration < kEps) return {query, s.transform_};

  /* compute factor that may be <0 or >1 */
  const double factor = toSeconds(query - s.time_) / duration;

  const Eigen::Vector3d p = s.transform_.translation() + factor * (e.transform_.translation() - s.transform_.translation());

  const Eigen::Quaterniond q_s(s.transform_.rotation());
  const Eigen::Quaterniond q_e(e.transform_.rotation());
  const Eigen::Quaterniond q = q_s.slerp(factor, q_e);

  Transform tf(q);
  tf.translation() = p;
  return {query, tf};
}

TimestampedTransform interpolate(const TimestampedTransform& start, const TimestampedTransform& end, const Time& time) {
  TimestampedTransform new_start;
  TimestampedTransform new_end;

  if (start.time_ > end.time_) {
    std::cout << "Interpolator: \n";
    std::cout << "Start time: " << toSecondsSinceFirstMeasurement(start.time_) << std::endl;
    std::cout << "End time: " << toSecondsSinceFirstMeasurement(end.time_) << std::endl;
    // throw std::runtime_error("transform interpolate:: start time is greater than end time");

    /*const Eigen::Vector3d origin = start.transform_.translation();
    const Eigen::Quaterniond rotation = Eigen::Quaterniond(start.transform_.rotation());
    Transform transform(rotation);
    transform.translation() = origin;

    return TimestampedTransform{time, transform};*/

    std::cout << "###############################################################################: \n";
    std::cout << "####################################### REPORT TO TURCAN ######################: \n";
    std::cout << "###############################################################################: \n";

    new_start.time_ = end.time_;
    new_end.time_ = start.time_;
    new_start.transform_ = end.transform_;
    new_end.transform_ = start.transform_;

  } else {
    new_start.time_ = start.time_;
    new_start.transform_ = start.transform_;
    new_end.time_ = end.time_;
    new_end.transform_ = end.transform_;
  }

  if (time > new_end.time_ || time < new_start.time_) {
    std::cout << "Interpolator: \n";
    std::cout << "Start time: " << toSecondsSinceFirstMeasurement(new_start.time_) << std::endl;
    std::cout << "End time: " << toSecondsSinceFirstMeasurement(new_end.time_) << std::endl;
    std::cout << "Query time: " << toSecondsSinceFirstMeasurement(time) << std::endl;
    throw std::runtime_error("transform interpolate:: query time is not between start and end time");
  }

  const double duration = toSeconds(new_end.time_ - new_start.time_);
  const double factor = toSeconds(time - new_start.time_) / (duration + 1e-6);  // avoid zero division
  const Eigen::Vector3d origin =
      new_start.transform_.translation() + (new_end.transform_.translation() - new_start.transform_.translation()) * factor;
  const Eigen::Quaterniond rotation =
      Eigen::Quaterniond(new_start.transform_.rotation()).slerp(factor, Eigen::Quaterniond(new_end.transform_.rotation()));
  Transform transform(rotation);
  transform.translation() = origin;

  return TimestampedTransform{time, transform};
}

Transform makeTransform(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
  Transform transform(q);
  transform.translation() = p;
  return transform;
}

} /* namespace o3d_slam */
