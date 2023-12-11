#pragma once

#include <sophus/se3.hpp>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/local_parameterization.h>

namespace sophus_se3 {

////from https://github.com/adrelino/Sophus/blob/master/test/ceres/local_parameterization_se3.hpp
struct SophusSE3Plus{
    template<typename T>
    bool operator()(const T* x_raw, const T* delta_raw, T* x_plus_delta_raw) const {
        const Eigen::Map< const Sophus::SE3<T> > x(x_raw);
        const Eigen::Map< const Eigen::Matrix<T,6,1> > delta(delta_raw);
        Eigen::Map< Sophus::SE3<T> > x_plus_delta(x_plus_delta_raw);
        x_plus_delta = x * Sophus::SE3<T>::exp(delta);
        return true;
      }
};


//https://github.com/strasdat/Sophus/blob/develop/test/ceres/local_parameterization_se3.hpp
class LocalParameterizationSE3 : public ceres::LocalParameterization {
public:
  virtual ~LocalParameterizationSE3() {}

  /**
   * \brief SE3 plus operation for Ceres
   *
   * \f$ T\cdot\exp(\widehat{\delta}) \f$
   */
  virtual bool Plus(const double * T_raw, const double * delta_raw,
                    double * T_plus_delta_raw) const {
    const Eigen::Map<const Sophus::SE3d> T(T_raw);
    const Eigen::Map<const Eigen::Matrix<double,6,1> > delta(delta_raw);
    Eigen::Map<Sophus::SE3d> T_plus_delta(T_plus_delta_raw);
    T_plus_delta = T * Sophus::SE3d::exp(delta);
    return true;
  }

  /**
   * \brief Jacobian of SE3 plus operation for Ceres
   *
   * \f$ \frac{\partial}{\partial \delta}T\cdot\exp(\widehat{\delta})|_{\delta=0} \f$
   */
  
  virtual bool ComputeJacobian(const double * T_raw, double * jacobian_raw)
    const {
    const Eigen::Map<const Sophus::SE3d> T(T_raw);
    Eigen::Map<Eigen::Matrix<double,6,7> > jacobian(jacobian_raw);
    jacobian = T.Dx_this_mul_exp_x_at_0().transpose();
  
    return true;
  }
  

  virtual int GlobalSize() const {
    return Sophus::SE3d::num_parameters;
  }

  virtual int LocalSize() const {
    return Sophus::SE3d::DoF;
  }
};


//https://groups.google.com/forum/#!topic/ceres-solver/a9JhUIWOn1I
static ceres::LocalParameterization* getParameterization(bool automaticDiff){
    if(automaticDiff){
        //std::cout<<"automatic diff sophusSE3 local parameterization"<<std::endl;
        return new ceres::AutoDiffLocalParameterization<SophusSE3Plus, 7, 6>;
    }
    else{
        //std::cout<<"analytic diff sophusSE3 local parameterization"<<std::endl;
        return new LocalParameterizationSE3();
    }
}



} //end ns sophus_se3

