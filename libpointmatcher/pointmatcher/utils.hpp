#pragma once

// C++ standard library
#include <string>
#include <vector>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/autodiff_local_parameterization.h>

#include <Eigen/Geometry>

struct DegeneracySolverOptions
{
bool isEnabled_{ true };
bool usePointToPoint_{ false };
bool usePointToPlane_{ true };
bool useBoundConstraints_{ false };
bool useSixDofRegularization_{ false };
bool useThreeDofRegularization_{ false };
float regularizationWeight_{ 0.0f };
};

// Assumes compact angle axis (3d vector where norm gives the angle) parameterization for rotations
// First 3 values of isometry_data are the compact angle axis, next 3 are the translation
template <typename T>
Eigen::Matrix<T, 6, 1> VectorFromIsometry3(const Eigen::Transform<T, 3, Eigen::Isometry>& isometry_3);

Eigen::Matrix<double, 6, 1> VectorFromIsometry3d(const Eigen::Isometry3d& isometry_3d);
// Assumes compact angle axis (3d vector where norm gives the angle) parameterization for rotations
// First 3 values of isometry_data are the compact angle axis, next 3 are the translation, last is scale
Eigen::Matrix<double, 7, 1> VectorFromAffine3d(const Eigen::Affine3d& affine_3d);
// Assumes compact angle axis (3d vector where norm gives the angle) parameterization for rotations
// First 3 values of isometry_data are the compact angle axis, next 3 are the translation
template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> Isometry3(const T* isometry_data);
// Assumes compact angle axis (3d vector where norm gives the angle) parameterization for rotations
// First 3 values of isometry_data are the compact angle axis, next 3 are the translation, last is scale
template <typename T>
Eigen::Transform<T, 3, Eigen::Affine> Affine3(const T* affine_data);

Eigen::Isometry3d Isometry3d(const Eigen::Matrix<double, 6, 1>& isometry_vector);

Eigen::Affine3d Affine3d(const Eigen::Matrix<double, 7, 1>& affine_vector);

template <typename T>
Eigen::Matrix<T, 6, 1> VectorFromIsometry3(const Eigen::Transform<T, 3, Eigen::Isometry>& isometry_3) {
  // Isometry3d linear().data() returns the data pointer to the full Isometry3d matrix rather than just the rotation
  const Eigen::Matrix<T, 3, 3> rotation = isometry_3.linear();
  Eigen::Matrix<T, 6, 1> isometry_3_vector;
  ceres::RotationMatrixToAngleAxis(rotation.data(), &(isometry_3_vector.data()[0]));
  isometry_3_vector[3] = isometry_3.translation().x();
  isometry_3_vector[4] = isometry_3.translation().y();
  isometry_3_vector[5] = isometry_3.translation().z();
  return isometry_3_vector;
}

template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> Isometry3(const T* isometry_data) {
  Eigen::Matrix<T, 3, 3> rotation;
  ceres::AngleAxisToRotationMatrix(isometry_data, rotation.data());
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> translation(&isometry_data[3]);
  Eigen::Transform<T, 3, Eigen::Isometry> isometry_3(Eigen::Transform<T, 3, Eigen::Isometry>::Identity());
  isometry_3.linear() = rotation;
  isometry_3.translation() = translation;
  return isometry_3;
}

template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> Isometry3(const T* rotation, const T* translation) {
  Eigen::Matrix<T, 3, 3> rotations;
  ceres::AngleAxisToRotationMatrix(rotation, rotations.data());
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> translations(&translation[3]);
  Eigen::Transform<T, 3, Eigen::Isometry> isometry_3(Eigen::Transform<T, 3, Eigen::Isometry>::Identity());
  isometry_3.linear() = rotations;
  isometry_3.translation() = translations;
  return isometry_3;
}

template <typename T>
Eigen::Transform<T, 3, Eigen::Affine> Affine3(const T* affine_data) {
  const Eigen::Transform<T, 3, Eigen::Isometry> isometry_3 = Isometry3(affine_data);
  const T scale = affine_data[6];
  Eigen::Transform<T, 3, Eigen::Affine> affine_3;
  affine_3.linear() = scale * isometry_3.linear();
  affine_3.translation() = isometry_3.translation();
  return affine_3;
}



struct SE3Plus {
  template <typename T>
  bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
    const Eigen::Transform<T, 3, Eigen::Isometry> pose = Isometry3(x);
    const Eigen::Transform<T, 3, Eigen::Isometry> pose_delta = Isometry3(delta);
    const Eigen::Transform<T, 3, Eigen::Isometry> updated_pose = pose * pose_delta;
    const Eigen::Matrix<T, 6, 1> updated_pose_vector = VectorFromIsometry3(updated_pose);
    for (int i = 0; i < 6; ++i) {
      x_plus_delta[i] = updated_pose_vector[i];
    }
    return true;
  }
};

/*
Eigen::Isometry3d axisAngleToIso(const double* cam){
    Eigen::Isometry3d poseFinal = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d rot;
    ceres::AngleAxisToRotationMatrix(cam,rot.data());
    poseFinal.linear() = rot;
    poseFinal.translation() = Eigen::Vector3d(cam[3],cam[4],cam[5]);
    return poseFinal;//.cast<float>();
}

Eigen::Isometry3d eigenQuaternionToIso(const Eigen::Quaterniond& q, const Eigen::Vector3d& t){
    Eigen::Isometry3d poseFinal = Eigen::Isometry3d::Identity();
    poseFinal.linear() = q.toRotationMatrix();
    poseFinal.translation() = t;
    return poseFinal;//.cast<float>();
}
*/

using SE3LocalParameterization = ceres::AutoDiffLocalParameterization<SE3Plus, 6, 6>;
