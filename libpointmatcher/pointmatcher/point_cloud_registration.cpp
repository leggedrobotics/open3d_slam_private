#include "point_cloud_registration.hpp"
#include "sophus_se3.hpp"
#include <sophus/se3.hpp>

PointCloudRegistrationCeres::PointCloudRegistrationCeres(const Eigen::Matrix<float, -1, -1>& source_cloud,
                                               const Eigen::Matrix<float, -1, -1>& target_cloud,
                                               const Eigen::Matrix<float, -1, -1>& source_normal,
                                               const Eigen::Matrix<float, -1, -1>& target_normal,
                                               Eigen::Matrix<float, 6, 6> degenerateEigenVectors,
                                               DegeneracySolverOptions& degenOptions)
    : error_terms_(), degenerateEigenVector_(degenerateEigenVectors) {
  //Eigen::Isometry3d initial_target_T_source_estimate = Eigen::Isometry3d::Identity();
  //initial_target_T_source_estimate.matrix() = initial_guess.matrix();
  //target_T_source_ = VectorFromIsometry3d(initial_target_T_source_estimate);
  //error_terms_.reserve(source_cloud.size());

  // Pointer is deleted by ceres an a new one is generated.
  problem_.reset(new ceres::Problem());

  // For now these 2 options are used together. Onky very slight computational overhead.
  if ((degenOptions.useSixDofRegularization_) || (degenOptions.useThreeDofRegularization_))
  {
    // Generate and assign degeneracy regularization rediduals.
    for (Eigen::Index j = 0; j < degenerateEigenVectors.cols(); j++){
      //std::cout << " degenerateEigenVectors.col(j): " << std::endl << degenerateEigenVectors.col(j) << std::endl;
      Eigen::Matrix<double, 6, 1> localEig = degenerateEigenVectors.col(j).template cast<double>();
      if ((localEig != Eigen::Matrix<double, 6, 1>::Zero(6, 1)))
      {
        if(degenOptions.useSymmetricPointToPlane_){
          /*std::cout << " Degeneracy bound added" << localEig << std::endl;
          std::cout << " Regularization Weight: " << degenOptions.regularizationWeight_ << std::endl;
          SymmetricDegeneracyCost* error_termDegen = new SymmetricDegeneracyCost(localEig, degenOptions.regularizationWeight_);
          problem_->AddResidualBlock(new ceres::AutoDiffCostFunction<SymmetricDegeneracyCost, 1, 6>(error_termDegen), NULL,
                                      state_.data());*/
          std::cout << " CURRENT DISABLED " << std::endl<< localEig << std::endl;
        }else{
          std::cout << " Degeneracy bound added " << std::endl<< localEig << std::endl;
          //float realWeight = std::pow(degenOptions.regularizationWeight_, 2);
          float realWeight = degenOptions.regularizationWeight_;

          if (realWeight > 0.0001)
          {
            std::cout << " Regularization Weight: " << degenOptions.regularizationWeight_ << std::endl;
            std::cout << " Real Regularization Weight: " << realWeight << std::endl;
            DegeneracyCost* error_termDegen = new DegeneracyCost(localEig, degenOptions.regularizationWeight_);
            ceres::LossFunction* data_loss = new ceres::ScaledLoss(new ceres::TrivialLoss(), realWeight, ceres::TAKE_OWNERSHIP);
            problem_->AddResidualBlock(new ceres::AutoDiffCostFunction<DegeneracyCost, 1, 3, 3>(error_termDegen), data_loss,
                                        translation_.data(), angleAxis_.data());
          }
        }
                                    
        //InequalityConstraint* error_termDegen = new InequalityConstraint(localEig);
        //problem_->AddResidualBlock(new ceres::AutoDiffCostFunction<InequalityConstraint, 1, 1, 3, 3>(error_termDegen), NULL,
        //                            residualWeight_.data(), translation_.data(), angleAxis_.data());
      }
    }
  }
  
  /*
  Eigen::Vector3d target_normal_converted = target_normal.col(0).template cast<double>();
 
  std::cout << "eigenVector 0 : " << eigenVector.row(0).col(0).value() << std::endl;
  std::cout << "eigenVector 1 : " << eigenVector.row(1).col(0).value() << std::endl;
  std::cout << "eigenVector 2 : " << eigenVector.row(2).col(0).value() << std::endl;
  std::cout << "eigenVector 3 : " << eigenVector.row(3).col(0).value() << std::endl;
  std::cout << "eigenVector 4 : " << eigenVector.row(4).col(0).value() << std::endl;
  std::cout << "eigenVector 5 : " << eigenVector.row(5).col(0).value() << std::endl;
 */

//#pragma omp parallel for
  for (size_t i = 0u; i < source_cloud.cols(); i++) {
    Eigen::Vector3d source_ = source_cloud.col(i).topRows(3).template cast<double>();
    Eigen::Vector3d target_ = target_cloud.col(i).topRows(3).template cast<double>();
    Eigen::Vector3d source_normal_ = source_normal.col(i).template cast<double>();
    Eigen::Vector3d target_normal_ = target_normal.col(i).template cast<double>();

    //ErrorTermWithNormals* error_term = new ErrorTermWithNormals(source_, source_normal_, target_, target_normal_);
    //error_terms_.push_back(error_term);
    ceres::LossFunction* no_loss = new ceres::TrivialLoss();
    //ceres::LossFunction* huber_loss = new ceres::HuberLoss(0.5);


    // SOPHUS

    if ((degenOptions.useSophusParametrization_) || (degenOptions.useSophusAutoDiffParametrization_))
    {
      ErrorTermWithNormalsSophus* error_term = new ErrorTermWithNormalsSophus(source_, target_, target_normal_);
      problem_->AddResidualBlock(new ceres::AutoDiffCostFunction<ErrorTermWithNormalsSophus, 1, 7>(error_term), NULL,
                                soph_.data());

    }

    if ((degenOptions.useAngleAxisParametrization_) && (degenOptions.usePointToPlane_))
    {
    // Point to plane (angle axis, translation and rotation separete 1x3 )
    ErrorTermWithNormalsAngleAxis* error_term = new ErrorTermWithNormalsAngleAxis(source_, target_, target_normal_);
    problem_->AddResidualBlock(new ceres::AutoDiffCostFunction<ErrorTermWithNormalsAngleAxis, 1, 3, 3>(error_term), no_loss,
                               translation_.data(), angleAxis_.data());
    }


    if ((degenOptions.useAngleAxisParametrization_) && (degenOptions.usePointToPoint_))
    {
      // Point to point (angle axis, translation and rotation separete 1x3 )
      ErrorTermPointToPointAngleAxis* error_term = new ErrorTermPointToPointAngleAxis(source_, target_, target_normal_);
      problem_->AddResidualBlock(new ceres::AutoDiffCostFunction<ErrorTermPointToPointAngleAxis, 3, 3, 3>(error_term), NULL,
                                translation_.data(), angleAxis_.data());
    }


    if ((degenOptions.useAngleAxisParametrization_) && (degenOptions.useSymmetricPointToPlane_))
    {
    // Symmetric Point to Plane
    //problem_->AddResidualBlock(new ceres::AutoDiffCostFunction<ErrorTermWithNormals, 2, 6>(error_term), NULL,
    //                        state_.data());
      ErrorTermWithSymmetricNormalsAngleAxis* error_term = new ErrorTermWithSymmetricNormalsAngleAxis(source_, target_, target_normal_, source_normal_);
      problem_->AddResidualBlock(new ceres::AutoDiffCostFunction<ErrorTermWithSymmetricNormalsAngleAxis, 2, 6>(error_term), NULL,
                                state_.data());
    }
  }
 
  // weight_updater_callback_.reset(new WeightUpdaterCallback(&data_association_, &parameters_, &error_terms_,
  //                                                         &weight_updater_, rotation_, translation_));
  //(*weight_updater_callback_)(ceres::IterationSummary());
}

void PointCloudRegistrationCeres::solve(ceres::Solver::Options options, ceres::Solver::Summary* summary, DegeneracySolverOptions& degenOptions, Eigen::Matrix<float, -1, -1> constraintProjectionMatrix) {
  // options.callbacks.push_back(weight_updater_callback_.get());

  // std::vector<int> constant_indicies({0, 1, 2, 3, 4, 5});
  // Holds the indicies of the parameters that are constant
  // ceres::SubsetParameterization* subset_parameterization = new ceres::SubsetParameterization(6, constant_indicies);
  // problem_->SetParameterization(target_T_source_.data(), subset_parameterization);
  // problem_->SetParameterBlockConstant(&(target_T_source_.data())[5]);

    /*
    std::cout << "constraintProjectionMatrix row0 : " << constraintProjectionMatrix.row(0) << std::endl;
    std::cout << "constraintProjectionMatrix row1 : " << constraintProjectionMatrix.row(1) << std::endl;
    std::cout << "constraintProjectionMatrix row2 : " << constraintProjectionMatrix.row(2) << std::endl;
    std::cout << "constraintProjectionMatrix row3 : " << constraintProjectionMatrix.row(3) << std::endl;
    std::cout << "constraintProjectionMatrix row4 : " << constraintProjectionMatrix.row(4) << std::endl;
    std::cout << "constraintProjectionMatrix row5 : " << constraintProjectionMatrix.row(5) << std::endl;
    */

  // MAKES IT SLOW
  //ceres::LocalParameterization* se3_local_parameterization = new SE3LocalParameterization;
  //problem_->SetParameterization(state_.data(), se3_local_parameterization);

  // SOPHUS
  //ceres::LocalParameterization* se3_local_parameterization_from_sophus = sophus_se3::getParameterization(false);
  //problem_->SetParameterization(soph_.data(), se3_local_parameterization_from_sophus);
    if ((degenOptions.useAngleAxisParametrization_) && (degenOptions.useBoundConstraints_))
    {
      //if (degenOptions.useThreeDofRegularization_ || degenOptions.useSixDofRegularization_)
      //{
      //  return;
      //}
      /*
      Eigen::Matrix<float, -1, -1> pinv;
      Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<float, -1, -1>> cqr(constraintProjectionMatrix);
      std::cout << "is invertible: " << cqr.isInvertible() << std::endl;
      pinv = cqr.pseudoInverse();

      std::cout << "pinv row0 : " << pinv.row(0) << std::endl;
      std::cout << "pinv row1 : " << pinv.row(1) << std::endl;
      std::cout << "pinv row2 : " << pinv.row(2) << std::endl;
      std::cout << "pinv row3 : " << pinv.row(3) << std::endl;
      std::cout << "pinv row4 : " << pinv.row(4) << std::endl;
      std::cout << "pinv row5 : " << pinv.row(5) << std::endl;

      
      // Setting inequality constraints
      problem_->SetParameterLowerBound(translation_.data(), 0, -2.0);
      problem_->SetParameterLowerBound(translation_.data(), 1, -2.0);
      problem_->SetParameterLowerBound(translation_.data(), 2, -2.0);

      problem_->SetParameterLowerBound(angleAxis_.data(), 0, -2.0);
      problem_->SetParameterLowerBound(angleAxis_.data(), 1, -2.0);
      problem_->SetParameterLowerBound(angleAxis_.data(), 2, -15.0);

      problem_->SetParameterUpperBound(translation_.data(), 0, 2.0);
      problem_->SetParameterUpperBound(translation_.data(), 1, 2.0);
      problem_->SetParameterUpperBound(translation_.data(), 2, 2.0);

      problem_->SetParameterUpperBound(angleAxis_.data(), 0, 2.0);
      problem_->SetParameterUpperBound(angleAxis_.data(), 1, 2.0);
      problem_->SetParameterUpperBound(angleAxis_.data(), 2, -14.8);
      */

      // std::string error;
      // std::cout << "is false: " << problem_->IsFeasible(&error) << std::endl;
      // std::cout << "infeasible bound: " << error.find("infeasible bound") << std::endl;
    }

  // This should be always true if you want to do operations in between.
  options.update_state_every_iteration = true;
  ceres::Solve(options, problem_.get(), summary);
}

Eigen::Affine3d PointCloudRegistrationCeres::transformationSophus() {

  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso = sophusToIsoTurcan(soph_);

  Eigen::Affine3d affine( iso.matrix() );

  return affine;
}

Eigen::Affine3d PointCloudRegistrationCeres::transformation() {
  Eigen::Affine3d affine = Eigen::Affine3d::Identity();
  affine.matrix() = Isometry3d(state_).matrix();
  
  // Eigen::Quaternion<double> estimated_rot(rotation_[0], rotation_[1], rotation_[2], rotation_[3]);
  // estimated_rot.normalize();
  // affine.rotate(estimated_rot);
  // affine.pretranslate(Eigen::Vector3d(translation_));
  return affine;
}

Eigen::Affine3d PointCloudRegistrationCeres::transformationSeparate() {
  
  Eigen::Affine3d affine = Eigen::Affine3d::Identity();
  state_.block<3,1>(0,0) = angleAxis_;
  state_.block<3,1>(3,0) = translation_;
  affine.matrix() = Isometry3d(state_).matrix();
  
  // Eigen::Quaternion<double> estimated_rot(rotation_[0], rotation_[1], rotation_[2], rotation_[3]);
  // estimated_rot.normalize();
  // affine.rotate(estimated_rot);
  // affine.pretranslate(Eigen::Vector3d(translation_));
  return affine;
}