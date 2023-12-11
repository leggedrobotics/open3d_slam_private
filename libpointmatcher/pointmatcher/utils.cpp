/*!
 * @file
 * @author Turcan Tuna (ANYbotics)
 * @date   29/06/2021
 * @brief  The implementation of the utilization functions of the slam_investigation class.
 */

//! Header
#include "utils.hpp"

// C++ standard library
#include <filesystem>


Eigen::Matrix<double, 6, 1> VectorFromIsometry3d(const Eigen::Isometry3d& isometry_3d) {
  return VectorFromIsometry3<double>(isometry_3d);
}

Eigen::Matrix<double, 7, 1> VectorFromAffine3d(const Eigen::Affine3d& affine_3d) {
  Eigen::Matrix3d rotation;
  Eigen::Matrix3d scale_matrix;
  affine_3d.computeRotationScaling(&rotation, &scale_matrix);
  // Assumes uniform scaling, which is the case for Affine3d
  const double scale = scale_matrix(0, 0);
  Eigen::Matrix<double, 7, 1> affine_3d_vector;
  ceres::RotationMatrixToAngleAxis(rotation.data(), &(affine_3d_vector.data()[0]));
  affine_3d_vector.block<3, 1>(3, 0) = affine_3d.translation();
  affine_3d_vector(6, 0) = scale;
  return affine_3d_vector;
}

Eigen::Isometry3d Isometry3d(const Eigen::Matrix<double, 6, 1>& isometry_vector) {
  return Isometry3(isometry_vector.data());
}

Eigen::Affine3d Affine3d(const Eigen::Matrix<double, 7, 1>& affine_vector) {
  return Affine3(affine_vector.data());
}