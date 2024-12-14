#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include "PointMatcher.h"
#include "PointMatcherPrivate.h"
#include "utils.hpp"

class ErrorTermWithNormalsAngleAxis {
 public:
  ErrorTermWithNormalsAngleAxis(const Eigen::Vector3d source_point, const Eigen::Vector3d target_point, const Eigen::Vector3d target_normal)
      : source_point_(source_point),
        target_point_(target_point),
        target_normal_(target_normal) {}

  template <typename T>
  bool operator()(const T* translation, const T* angleAxis, T* residuals) const {
    
    // Point to plane angle axis
    T p[3] = {T(source_point_[0]), T(source_point_[1]), T(source_point_[2])};
    ceres::AngleAxisRotatePoint(angleAxis,p,p);

    p[0] += translation[0];
    p[1] += translation[1];
    p[2] += translation[2];

    // Can this be optimized?
    // The error is the difference between the predicted and observed position.
    residuals[0] = (p[0] - T(target_point_[0])) * T(target_normal_[0]) + \
                    (p[1] - T(target_point_[1])) * T(target_normal_[1]) + \
                    (p[2] - T(target_point_[2])) * T(target_normal_[2]);

    return true;
  }

 private:
  Eigen::Vector3d source_point_;
  Eigen::Vector3d target_point_;
  Eigen::Vector3d target_normal_;
};

class ErrorTermPointToPointAngleAxis {
 public:
  ErrorTermPointToPointAngleAxis(const Eigen::Vector3d source_point, const Eigen::Vector3d target_point, const Eigen::Vector3d target_normal)
      : source_point_(source_point),
        target_point_(target_point),
        target_normal_(target_normal) {}

  template <typename T>
  bool operator()(const T* translation, const T* angleAxis, T* residuals) const {
    
    // Point to plane angle axis
    T p[3] = {T(source_point_[0]), T(source_point_[1]), T(source_point_[2])};
    ceres::AngleAxisRotatePoint(angleAxis,p,p);

    p[0] += translation[0];
    p[1] += translation[1];
    p[2] += translation[2];

    // Can this be optimized?
    // The error is the difference between the predicted and observed position.
    residuals[0] = (p[0] - T(target_point_[0]));
    residuals[1] = (p[1] - T(target_point_[1]));
    residuals[2] = (p[2] - T(target_point_[2])); 

    return true;
  }

 private:
  Eigen::Vector3d source_point_;
  Eigen::Vector3d target_point_;
  Eigen::Vector3d target_normal_;
};

class ErrorTermWithSymmetricNormalsAngleAxis {
 public:
  ErrorTermWithSymmetricNormalsAngleAxis(const Eigen::Vector3d source_point, const Eigen::Vector3d target_point, const Eigen::Vector3d target_normal, const Eigen::Vector3d source_normal)
      : source_point_(source_point),
        target_point_(target_point),
        target_normal_(target_normal),
        source_normal_(source_normal) {}

  template <typename T>
  bool operator()(const T* transformationVector, T* residuals) const {
    
    // Symmetric Point to point
    const auto target_T_source = Isometry3<T>(transformationVector);
    // Compute error
    const Eigen::Matrix<T, 3, 1> estimated_target_t_point = target_T_source * source_point_.cast<T>();
    const Eigen::Matrix<T, 3, 1> target_F_point_t_estimated_point =
      estimated_target_t_point - target_point_.cast<T>();
    const Eigen::Matrix<T, 3, 1> estimated_source_t_point = target_T_source.inverse() * target_point_.cast<T>();
    const Eigen::Matrix<T, 3, 1> source_F_point_t_estimated_point =
      estimated_source_t_point - source_point_.cast<T>();
    residuals[0] = source_F_point_t_estimated_point.dot(source_normal_.cast<T>());
    residuals[1] = target_F_point_t_estimated_point.dot(target_normal_.cast<T>());
    return true;
    
  }

 private:
  Eigen::Vector3d source_point_;
  Eigen::Vector3d target_point_;
  Eigen::Vector3d target_normal_;
  Eigen::Vector3d source_normal_;
};

class DegeneracyCost {
 public:
  DegeneracyCost(const Eigen::Matrix<double, 6, 1>& eigenVector, const double weight)
      : eigenVector_(eigenVector),
      weight_(weight) {}

  template <typename T>
  bool operator()(const T* translation, const T* angleAxis, T* residuals) const {
    
    // Weight is applied as scaled loss.
    residuals[0] = ((T(angleAxis[0])) * T(eigenVector_[0]) + \
                    (T(angleAxis[1])) * T(eigenVector_[1]) + \
                    (T(angleAxis[2])) * T(eigenVector_[2])+ \
                    (T(translation[0])) * T(eigenVector_[3])+ \
                    (T(translation[1])) * T(eigenVector_[4])+ \
                    (T(translation[2])) * T(eigenVector_[5]));   
    return true;
  }

 private:
  Eigen::Matrix<double, 6, 1> eigenVector_;
  double weight_{0.0};
};