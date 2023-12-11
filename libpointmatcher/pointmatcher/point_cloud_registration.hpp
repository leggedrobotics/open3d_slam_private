#pragma once

#include <vector>

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "PointMatcher.h"
#include "PointMatcherPrivate.h"

#include "error_term.hpp"

#include "utils.hpp"

class PointCloudRegistrationCeres{

 public:
  PointCloudRegistrationCeres(const Eigen::Matrix<float, -1, -1>& source_cloud,
                                               const Eigen::Matrix<float, -1, -1>& target_cloud,
                                               const Eigen::Matrix<float, -1, -1>& source_normal,
                                               const Eigen::Matrix<float, -1, -1>& target_normal,
                                               Eigen::Matrix<float, 6, 6> eigenVector,
                                               DegeneracySolverOptions& degenOptions);

  void solve(ceres::Solver::Options options, ceres::Solver::Summary* Summary, DegeneracySolverOptions& degenOptions, Eigen::Matrix<float, -1, -1> constraintProjectionMatrix);
  Eigen::Affine3d transformation();
  Eigen::Affine3d transformationSophus();
  Eigen::Affine3d transformationSeparate();


  Sophus::SE3d isoToSophusTurcan(const Eigen::Isometry3d& pose){
    Sophus::SE3d sophusPose = Sophus::SE3d(pose.linear(), pose.translation());
    //setRotationMatrix
      return sophusPose;
  }

  Eigen::Isometry3d sophusToIsoTurcan(Sophus::SE3d soph){
      //    return Isometry3d(soph.matrix());
      Eigen::Isometry3d poseFinal = Eigen::Isometry3d::Identity();
      poseFinal.linear() = soph.rotationMatrix();
      poseFinal.translation() = soph.translation();
      return poseFinal;
  }

 private:
  std::vector<ErrorTermWithNormals*> error_terms_;
  std::unique_ptr<ceres::Problem> problem_;
  Eigen::Matrix<double, 6, 1> state_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 3, 1> translation_ = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Matrix<double, 3, 1> angleAxis_ = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Matrix<double, 1, 1> residualWeight_ = Eigen::Matrix<double, 1, 1>::Constant(1, 1, 1);
  
  Eigen::Matrix<float, 6, 6> degenerateEigenVector_ = Eigen::Matrix<float, 6, 6>::Zero();


  Sophus::SE3d soph_ = isoToSophusTurcan(Eigen::Isometry3d::Identity());
  // ProbabilisticWeights weight_updater_;
  // std::unique_ptr<WeightUpdaterCallback> weight_updater_callback_;
};