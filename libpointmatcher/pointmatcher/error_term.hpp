#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
//#include <pcl/point_types.h>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "PointMatcher.h"
#include "PointMatcherPrivate.h"

#include "utils.hpp"

class ErrorTerm {
 public:
  static const int kResiduals = 3;
  ErrorTerm(const Eigen::Vector3d source_point, const Eigen::Vector3d target_point)
      : source_point_(source_point),
        target_point_(target_point),
        weight_(new ceres::LossFunctionWrapper(new ceres::ScaledLoss(NULL, 1, ceres::TAKE_OWNERSHIP), ceres::TAKE_OWNERSHIP)) {}

  template <typename T>
  bool operator()(const T* const rotation, const T* const translation, T* residuals) const {
    T point_x[kResiduals];
    T point_y[kResiduals];
    for (int i = 0; i < kResiduals; i++) {
      point_x[i] = T(source_point_[i]);
      point_y[i] = T(target_point_[i]);
    }
    T transformed_point[kResiduals];
    ceres::QuaternionRotatePoint(rotation, point_x, transformed_point);
    for (int i = 0; i < kResiduals; i++) {
      transformed_point[i] = transformed_point[i] + translation[i];
      residuals[i] = point_y[i] - transformed_point[i];
    }
    return true;
  }

  void updateWeight(double new_weight) {
    weight_->Reset(new ceres::ScaledLoss(NULL, new_weight, ceres::TAKE_OWNERSHIP), ceres::TAKE_OWNERSHIP);
  }

  ceres::LossFunctionWrapper* weight() { return weight_; }

 private:
  Eigen::Vector3d source_point_;
  Eigen::Vector3d target_point_;
  ceres::LossFunctionWrapper* weight_;
};

class ErrorTermWithNormals {
 public:
  ErrorTermWithNormals(const Eigen::Vector3d source_point, const Eigen::Vector3d source_normal, const Eigen::Vector3d target_point, const Eigen::Vector3d target_normal)
      : source_point_(source_point),
        target_point_(target_point),
        source_normal_(source_normal),
        target_normal_(target_normal) {}

  template <typename T>
  bool operator()(const T* transformationVector, T* residuals) const {
    
    /*
    // Point to plane angle axis
    T p[3] = {T(source_point_[0]), T(source_point_[1]), T(source_point_[2])};
    ceres::AngleAxisRotatePoint(transformationVector,p,p);

    // camera[3,4,5] are the translation.
    p[0] += transformationVector[3];
    p[1] += transformationVector[4];
    p[2] += transformationVector[5];

    // The error is the difference between the predicted and observed position.
    residuals[0] = (p[0] - T(target_point_[0])) * T(target_normal_[0]) + \
                    (p[1] - T(target_point_[1])) * T(target_normal_[1]) + \
                    (p[2] - T(target_point_[2])) * T(target_normal_[2]);

    return true;
    */

    /*
    // Point to plane
    // This is a residual template. Needs to be written explicitly for each input (i.e. rotation and translation) for correct jacobian
    // calculation.
    const auto target_T_source = Isometry3<T>(transformationVector);
    // Compute error
    const Eigen::Matrix<T, 3, 1> estimated_target_t_point = target_T_source * source_point_.cast<T>();
    const Eigen::Matrix<T, 3, 1> target_F_point_t_estimated_point = estimated_target_t_point - target_point_.cast<T>();
    residuals[0] = target_F_point_t_estimated_point.dot(target_normal_.cast<T>());
    return true;
    */

    /*
    // Point to point
    const auto target_T_source = Isometry3<T>(transformationVector);
    // Compute error
    const Eigen::Matrix<T, 3, 1> estimated_target_t_point = target_T_source * source_point_.cast<T>();
    residuals[0] = estimated_target_t_point[0] - target_point_[0];
    residuals[1] = estimated_target_t_point[1] - target_point_[1];
    residuals[2] = estimated_target_t_point[2] - target_point_[2];
    return true;
    */
    /*
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
    */
  }

 private:
  Eigen::Vector3d source_point_;
  Eigen::Vector3d target_point_;
  Eigen::Vector3d source_normal_;
  Eigen::Vector3d target_normal_;
};


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

class ErrorTermWithNormalsSophus {
 public:
  ErrorTermWithNormalsSophus(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point, const Eigen::Vector3d& target_normal)
      : source_point_(source_point),
        target_point_(target_point),
        target_normal_(target_normal) {}

  template <typename T>
  bool operator()(const T* transformationVector, T* residuals) const {

    // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
    Eigen::Matrix<T,3,1> p; 
    p << T(source_point_[0]), T(source_point_[1]), T(source_point_[2]);
    Eigen::Matrix<T,3,1> targetP; 
    targetP << T(target_point_[0]), T(target_point_[1]), T(target_point_[2]);

    //Eigen::Matrix<T,3,1> normal; normal << T(target_normal_[0]), T(target_normal_[1]), T(target_normal_[2]);

    // Map the T* array to an Sophus SE3 object (with appropriate Scalar type)
    Sophus::SE3<T> q = Eigen::Map< const Sophus::SE3<T> >(transformationVector);

    // Rotate the point using Eigen rotations
    p = q.unit_quaternion() * p + q.translation();

    // The error is the difference between the predicted and observed position projected onto normal
    residuals[0] = (p - targetP).dot(target_normal_.cast<T>());
    /*
    residuals[0] = (p[0] - T(target_point_[0])) * T(target_normal_[0]) + \
                    (p[1] - T(target_point_[1])) * T(target_normal_[1]) + \
                    (p[2] - T(target_point_[2])) * T(target_normal_[2]);
                    */
    return true;
  }

 private:
  Eigen::Vector3d source_point_;
  Eigen::Vector3d target_point_;
  Eigen::Vector3d target_normal_;
};

class DegeneracyCost {
 public:
  DegeneracyCost(const Eigen::Matrix<double, 6, 1>& eigenVector, const double weight)
      : eigenVector_(eigenVector),
      weight_(weight) {}

  template <typename T>
  bool operator()(const T* translation, const T* angleAxis, T* residuals) const {
    
    //T weightLocal = T(weight_);  T(weight_) *
    //T(weight_) * 
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

class SymmetricDegeneracyCost {
 public:
  SymmetricDegeneracyCost(const Eigen::Matrix<double, 6, 1>& eigenVector, const double weight)
      : eigenVector_(eigenVector),
      weight_(weight) {}

  template <typename T>
  bool operator()(const T* transformation, T* residuals) const {
    
    //T weightLocal = T(weight_);
    residuals[0] = T(weight_) * ((T(transformation[0])) * T(eigenVector_[0]) + \
                    (T(transformation[1])) * T(eigenVector_[1]) + \
                    (T(transformation[2])) * T(eigenVector_[2])+ \
                    (T(transformation[3])) * T(eigenVector_[3])+ \
                    (T(transformation[4])) * T(eigenVector_[4])+ \
                    (T(transformation[5])) * T(eigenVector_[5]));   
    return true;
  }

 private:
  Eigen::Matrix<double, 6, 1> eigenVector_;
  double weight_{0.0};
};

class InequalityConstraint {
 public:
  InequalityConstraint(const Eigen::Matrix<double, 6, 1>& eigenVector)
      : eigenVector_(eigenVector) {}

  template <typename T>
  bool operator()(const T* scheduleWeight, const T* translation, const T* angleAxis, T* residuals) const {
    /*
    T reduction = ceres::sqrt(ceres::pow(T(angleAxis[0]),2) + \
                    ceres::pow(T(angleAxis[1]),2) + \
                    ceres::pow(T(angleAxis[2]),2)+ \
                    ceres::pow(T(translation[0]),2)+ \
                    ceres::pow(T(translation[1]),2)+ \
                    ceres::pow(T(translation[2]),2));
    std::cout << " reduction: " << reduction << std::endl;
    */

    // Amount of motion in the given eigenvector directoon. Basically dot product.
    residuals[0] = ((T(angleAxis[0])) * T(eigenVector_[0]) + \
                    (T(angleAxis[1])) * T(eigenVector_[1]) + \
                    (T(angleAxis[2])) * T(eigenVector_[2])+ \
                    (T(translation[0])) * T(eigenVector_[3])+ \
                    (T(translation[1])) * T(eigenVector_[4])+ \
                    (T(translation[2])) * T(eigenVector_[5]));
    
    if (residuals[0] > T(scheduleWeight[0]) * T(0.7))
    {
      return false;
    }

    if (residuals[0] < T(scheduleWeight[0]) * T(-0.7))
    {
      return false;
    }
    residuals[0] = residuals[0] * T(scheduleWeight[0]);
    return true;
  }

 private:
  Eigen::Matrix<double, 6, 1> eigenVector_;

};