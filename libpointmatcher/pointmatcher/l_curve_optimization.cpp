#include "l_curve_optimization.hpp"
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include "sophus_se3.hpp"
#include <sophus/se3.hpp>

LcurveOptimizer::LcurveOptimizer(Eigen::Matrix<float, -1, 6> A, Eigen::Matrix<float, -1, 1> b, int nbConstraints)
    : A_(A), b_(b), nbConstraints_(nbConstraints) {
}

void LcurveOptimizer::solveState(Eigen::Matrix<float, -1, 6> A_copy, Eigen::Matrix<float, -1, 1> b_copy){

  //  HERE  I NEED TO APPLY LAMBDA BEFORE SOLVING WE ARE LOOKING FOR X_LAMBDA
  A_copy.bottomLeftCorner(nbConstraints_, 6) = A_copy.bottomLeftCorner(nbConstraints_, 6) * lambda_;
  x_ = A_copy.template cast<double>().jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_copy.template cast<double>());
  //std::cout << "x_lambda solution FOUND: " << x_ << std::endl;

  //BOOST_AUTO(solverQR, A_copy.householderQr());
  //x_ = solverQR.solve(b_copy);

}

void LcurveOptimizer::evaluate() {

  // Prepare 
  //const Eigen::MatrixXd mT = A_.template cast<double>().transpose() * Eigen::Matrix<double, 6, 6>::Identity(6,6) * A_.template cast<double>() + lambda_ * L_.template cast<double>();
  //const Eigen::MatrixXd l_inverse = L_.template cast<double>().inverse();

  //std::cout << "Preparations are done" << std::endl;

  //Eigen::MatrixXd mL = l_inverse * mT * l_inverse;

  //luL_ = Eigen::FullPivLU<Eigen::MatrixXd>(mL);

  //std::cout << "PivLU done" << std::endl;

  //curveture_ = evalLCurveCurvature();
  //curvetureDerivative_ = evalLCurveCurvatureDerivative();

  //std::cout << "curveture calculated" << std::endl;

  //std::cout << "x_lambda solution: " << x_.transpose() << std::endl;
  //std::cout << "L_: " << L_.template cast<double>() << std::endl;
  //std::cout << "Curveture: " << curveture_ << std::endl;
  //std::cout << "Curveture Derivative: " << curvetureDerivative_ << std::endl;

  L_ = A_.block(6, 0, 6 + nbConstraints_, 6);

  lambda_n = 4000;
  int lambda_max = 20000;
  int lambda_min = 1;
  residualNorms.reserve(lambda_n);
  smoothnessNorms.reserve(lambda_n);
  sampledLambdas.reserve(lambda_n);
  curv.reserve(lambda_n);
  double h = std::pow(lambda_max / lambda_min,  1. / (lambda_n - 1));

  /**
   * Loop over regularization parameter values
   */

  auto tryLambdas = linspace(lambda_min, lambda_max, lambda_n);

  for (auto lambda : tryLambdas){
    //double lambda = lambda_min * std::pow(h, i);
    //std::cout << "[" << i + 1 << "/" << lambda_n << "]" << std::endl;
    //std::cout << "lambda = " << lambda << std::endl;
    //std::cout << "--------" << std::endl;
    /**
     * Setting regularization parameter
     */
    setLambda(lambda);
    /**
     * Solving the problem
     */
    solveState(A_, b_);
    /**
     * Collecting L-curve and L-curve curvature
     */
    /*
    std::cout << "############" << std::endl;
    std::cout << "Lambda: " << lambda << std::endl;
    std::cout << "Norm of the eq: " << evalEqNorm2() << std::endl;
    std::cout << "Norm of regularization: " << evalSmoothnessConstraintNorm2() << std::endl;
    std::cout << "############" << std::endl;
    */
    //std::cout << "isEqNorm2 inf: " << std::isinf(evalEqNorm2()) << std::endl;
    //std::cout << "smoothness inf: " << std::isinf(evalSmoothnessConstraintNorm2()) << std::endl;
    residualNorms.push_back(evalEqNorm2());
    smoothnessNorms.push_back(evalSmoothnessConstraintNorm2());
    sampledLambdas.push_back(lambda);
  }

  SPLINTER::DataTable samples;

  // Sample the function
  //SPLINTER::DenseVector x(2);

  //std::cout << "Added the sample" << std::endl;

  // Convert to log
  std::vector<double> logresidualNorms;
  logresidualNorms.reserve(residualNorms.size());
  for (auto residualNorm : residualNorms){

    if (residualNorm == 0.0)
    {
      // A dirty hack to avoid log10(0) = -inf
      continue;
    }

    logresidualNorms.push_back(std::log10(1 / residualNorm));
    //logresidualNorms.push_back(std::log10(residualNorm));
  }

  std::vector<double> logsmoothnessNorms;
  logsmoothnessNorms.reserve(smoothnessNorms.size());
  for (auto smoothnessNorm : smoothnessNorms){
    if (smoothnessNorm == 0.0)
    {
      // A dirty hack to avoid log10(0) = -inf
      continue;
    }
    
    logsmoothnessNorms.push_back(std::log10(1 / smoothnessNorm));
    //logsmoothnessNorms.push_back(std::log10(smoothnessNorm));
    //std::cout << "Value logsmoothness: " << std::log10(smoothnessNorm) << std::endl;
  }

  for(int j = 0; j < smoothnessNorms.size(); j++){
      // Store sample
      samples.add_sample(logsmoothnessNorms[j], logresidualNorms[j]);
  }

  //std::cout << "Converted to log" << std::endl;

  const auto [min, max] = std::minmax_element(begin(logsmoothnessNorms), end(logsmoothnessNorms));

  //std::cout << "Found min max" << std::endl;
  std::cout << "min: " << *min << std::endl;
  std::cout << "max: " << *max << std::endl;

  const unsigned int y_dim = 1;
  const std::vector<unsigned int> degrees = {3};
  //const std::vector<std::vector<double>> knots = { {-5.0, -5.0, -5.0, -5.0, 10.0, 10.0, 10.0, 10.0} };
  //const std::vector<std::vector<double>> knots = { {*min - 1.0, *min - 1.0, *min - 1.0, *min - 1.0, *max + 1.0, *max + 1.0, *max + 1.0, *max + 1.0} };
  double maxx = *max;
  double minn = *min;
  const std::vector<std::vector<double>> knots = {linspace(minn - (0.1), maxx + (0.1), 2000)};

  double stepsize = ( (maxx + 0.1) - (minn - 0.1) ) / (2000);

  SPLINTER::BSpline bspline3(degrees, knots, y_dim);
  bspline3.fit(samples);

  // Constructed thespline

  //SPLINTER::DenseVector x(1);
  // Doesn't necessarily have to be sampled in the same size as the original data ?
  auto x0_vec = linspace(minn + 5 * stepsize, maxx - 5 * stepsize, 15000);

  std::vector<double> y0_vec;
  y0_vec.reserve(x0_vec.size());

  std::vector<double> curvatureTry;
  curvatureTry.reserve(x0_vec.size());

  for (auto x0 : x0_vec)
  {
    y0_vec.push_back(bspline3.eval(std::vector<double>({x0})).at(0));

    //std::cout << "x0: " << x0 << std::endl;

    auto jacob = bspline3.eval_jacobian(std::vector<double>({x0}));
    auto hessian = bspline3.eval_hessian(std::vector<double>({x0}));

    double curv = std::abs(hessian.at(0).at(0).at(0)) / std::pow(1 + jacob.at(0).at(0) * jacob.at(0).at(0), 1.5);

    //std::cout << "curv: " << curv << std::endl;
    //std::cout << "jacob: " << jacob.at(0).at(0) << std::endl;
    //std::cout << "hessian: " << hessian.at(0).at(0).at(0) << std::endl;

    curvatureTry.push_back(curv);
  }
 

  cpp_curveture.reserve(curvatureTry.size());

  cpp_curveture = curvatureTry;

  cpp_curveture_x_axis.reserve(x0_vec.size());
  cpp_curveture_x_axis = x0_vec;

  //std::cout << "Jacob, hessian and curvature calculated" << std::endl;

  // Find the index of the maximum curvature.

  const auto [min_curve, max_curve] = std::minmax_element(begin(curvatureTry), end(curvatureTry));

  auto maxElementIndex = std::distance(curvatureTry.begin(), max_curve);

  std::cout << "Index of Max Curvature: " << maxElementIndex << std::endl;
  std::cout << "Maximum Curvature: " << *max_curve << std::endl;
  std::cout << "Minimum Curvature: " << *min_curve << std::endl;

  //
  //auto iteratorToTheClosestValueInLogSmoothness = closest(logsmoothnessNorms, x0_vec[maxElementIndex]);

  // Use the found index to identify the closest value in the original x axis.
  auto iteratorToTheClosestValueInLogSmoothness = std::lower_bound(logsmoothnessNorms.begin(), logsmoothnessNorms.end(), x0_vec[maxElementIndex]); 

  // Find the index of the closest value in the original x axis.
  auto indexOf = std::distance(logsmoothnessNorms.begin(), iteratorToTheClosestValueInLogSmoothness);

  std::cout << "Index of lambda: " << indexOf << std::endl;
  std::cout << "Value of lambda: " << sampledLambdas[indexOf] << std::endl;

  optimalRegularizationWeight_ = sampledLambdas[indexOf];

}

/*std::vector<double>::iterator LcurveOptimizer::closest(std::vector<double> const& vec, double value) {
    auto const it = std::lower_bound(vec.begin(), vec.end(), value);
    if (it == vec.end()) { return it; }

    return it;
}
*/

std::vector<double> LcurveOptimizer::denseToVec(const SPLINTER::DenseVector &dense)
{
    auto vec = std::vector<double>(dense.size());
    for(int i = 0; i < (int) dense.size(); i++) {
        vec.at(i) = dense(i);
    }

    return vec;
}

SPLINTER::DenseVector LcurveOptimizer::vecToDense(const std::vector<double> &vec)
{
    SPLINTER::DenseVector dense(vec.size());
    for(int i = 0; i < (int) vec.size(); i++) {
        dense(i) = vec.at(i);
    }

    return dense;
}

std::vector<double> LcurveOptimizer::linspace(double start, double stop, unsigned int num)
{
    std::vector<double> ret;
    double dx = 0;
    if (num > 1)
        dx = (stop - start)/(num-1);
    for (unsigned int i = 0; i < num; ++i)
        ret.push_back(start + i*dx);
    return ret;
}

std::vector<double> LcurveOptimizer::getLambdas(){

  return sampledLambdas;
}

std::vector<double> LcurveOptimizer::getResiduals(){

  //return cpp_curveture;
  return residualNorms;
}

double LcurveOptimizer::getOptimalRegularizationWeight(){

  return optimalRegularizationWeight_;
}

std::vector<double> LcurveOptimizer::getRegNorms(){

  //return cpp_curveture_x_axis;
  return smoothnessNorms;
}

int LcurveOptimizer::getNumberOfSamples() {
  return lambda_n;
}

double LcurveOptimizer::evalEqNorm2() {
  Eigen::VectorXd dv = A_.template cast<double>() * x_ - b_.template cast<double>();
  return dv.norm();
}

double LcurveOptimizer::evalSmoothnessConstraintNorm2() {
  //return x.dot(L_.template cast<double>() * x);
  //Eigen::Matrix<float, 1, 6> inter = L_.topLeftCorner(1, 6);
  return (L_.template cast<double>() * x_).norm();
}

double LcurveOptimizer::evalLCurveCurvature() {
  Eigen::VectorXd ds = -luL_.solve(x_);
  double dksi = evaldKsidLambda(ds);
  return -std::fabs(1. / dksi / std::pow(1. + lambda_ * lambda_, 1.5));
}

double LcurveOptimizer::evaldKsidLambda(const Eigen::VectorXd& ds) {
  return 2. * x_.dot(L_.template cast<double>() * ds);
}

double LcurveOptimizer::evalLCurveCurvatureDerivative() {
  Eigen::VectorXd ds = -luL_.solve(x_);
  Eigen::VectorXd d2s = -2. * luL_.solve(ds);
  double dksi = evaldKsidLambda(ds);
  double d2ksi = evald2Ksid2Lambda(ds, d2s);
  return -d2ksi * std::pow(dksi, -2.) * std::pow(1. + lambda_ * lambda_, -1.5) +
      -3. * lambda_ / dksi * std::pow(1. + lambda_ * lambda_, -2.5);
}

double LcurveOptimizer::evald2Ksid2Lambda(const Eigen::VectorXd& ds,
                                            const Eigen::VectorXd& d2s) {
  return 2. * ds.dot(L_.template cast<double>() * ds) + 2. * x_.dot(L_.template cast<double>() * d2s);
}


void LcurveOptimizer::setLambda(double lambda) {
  //std::cout << "setting lambda: " << lambda << std::endl;
  lambda_ = lambda; 
}

double LcurveOptimizer::getLcurveCurveture() {
  
  return curveture_;
}

double LcurveOptimizer::getLcurveCurvetureDerivative(){
  
  return curvetureDerivative_;
}


Eigen::Affine3d LcurveOptimizer::transformationSophus() {

  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso = sophusToIsoTurcan(soph_);

  Eigen::Affine3d affine( iso.matrix() );

  return affine;
}

Eigen::Affine3d LcurveOptimizer::transformation() {
  Eigen::Affine3d affine = Eigen::Affine3d::Identity();
  affine.matrix() = Isometry3d(state_).matrix();
  
  // Eigen::Quaternion<double> estimated_rot(rotation_[0], rotation_[1], rotation_[2], rotation_[3]);
  // estimated_rot.normalize();
  // affine.rotate(estimated_rot);
  // affine.pretranslate(Eigen::Vector3d(translation_));
  return affine;
}

Eigen::Affine3d LcurveOptimizer::transformationSeparate() {
  
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