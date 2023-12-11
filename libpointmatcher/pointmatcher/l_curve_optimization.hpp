#pragma once

#include <vector>

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "PointMatcher.h"
#include "PointMatcherPrivate.h"

#include "error_term.hpp"

#include "utils.hpp"
//#include <BSplineInterpolation/Interpolation.hpp>
//#include <datatable.h>
#include <bspline.h>
//#include <bsplinebuilder.h>

#include <bspline_utils.h>
#include <bspline_builders.h>



class LcurveOptimizer{ 

 public:
  LcurveOptimizer(Eigen::Matrix<float, -1, 6> A, Eigen::Matrix<float, -1, 1> b, int nbConstraints);

  // Evaluate the curveture of the L-Curve, the derivative of the curveture at that point.
  void evaluate();
  
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

  // Set the regularization Parameter lambda.
  void setLambda(double lambda);
  double getLcurveCurveture();
  double getLcurveCurvetureDerivative();

  int getNumberOfSamples();
  std::vector<double> getLambdas();
  std::vector<double> getResiduals();
  std::vector<double> getRegNorms();

  double getOptimalRegularizationWeight();

 private:
  std::vector<ErrorTermWithNormals*> error_terms_;
  std::unique_ptr<ceres::Problem> problem_;
  Eigen::Matrix<double, 6, 1> state_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 3, 1> translation_ = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Matrix<double, 3, 1> angleAxis_ = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Matrix<double, 1, 1> residualWeight_ = Eigen::Matrix<double, 1, 1>::Constant(1, 1, 1);
  
  Eigen::Matrix<float, -1, 6> A_;// = Eigen::Matrix<float, 6, 6>::Zero();
  Eigen::Matrix<float, -1, 1> b_;// = Eigen::Matrix<float, 6, 1>::Zero();


  Sophus::SE3d soph_ = isoToSophusTurcan(Eigen::Isometry3d::Identity());
  double evalLCurveCurvature();
  double evalLCurveCurvatureDerivative();
  double evaldKsidLambda(const Eigen::VectorXd& ds);
  double evald2Ksid2Lambda(const Eigen::VectorXd& ds, const Eigen::VectorXd& d2s);

  double evalSmoothnessConstraintNorm2();
  double evalEqNorm2();

  void solveState(Eigen::Matrix<float, -1, 6> A_copy, Eigen::Matrix<float, -1, 1> b_copy);
  std::vector<double> linspace(double start, double stop, unsigned int num);

  //std::vector<double>::iterator closest(std::vector<double> const& vec, double value);
  SPLINTER::DenseVector vecToDense(const std::vector<double> &vec);
  std::vector<double> denseToVec(const SPLINTER::DenseVector &dense);


  double lambda_ = 0.0;
  double curveture_ = 0.0;
  double curvetureDerivative_ = 0.0;
  int lambda_n = 1500;

  std::vector<double> residualNorms;
  std::vector<double> smoothnessNorms;
  std::vector<double> sampledLambdas;
  std::vector<double> curv;

  std::vector<double> cpp_curveture;
  std::vector<double> cpp_curveture_x_axis;


  Eigen::Matrix<double, 6, 1> x_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<float, -1, 6> L_;
  Eigen::FullPivLU<Eigen::MatrixXd> luL_;

  double optimalRegularizationWeight_ = 0.0;
  int nbConstraints_ = 1;

  // ProbabilisticWeights weight_updater_;
  // std::unique_ptr<WeightUpdaterCallback> weight_updater_callback_;
};