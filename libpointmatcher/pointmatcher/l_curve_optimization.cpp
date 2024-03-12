#include "l_curve_optimization.hpp"
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include "sophus_se3.hpp"
#include <sophus/se3.hpp>
#include <functional>

LcurveOptimizer::LcurveOptimizer(Eigen::Matrix<float, -1, 6> A, Eigen::Matrix<float, -1, 1> b, int nbConstraints)
    : A_(A), b_(b), nbConstraints_(nbConstraints) {

      //L_ = Eigen::Matrix<double, nbConstraints_, 6>::Zero()
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


  //std::cout << "Curveture: " << curveture_ << std::endl;
  //std::cout << "Curveture Derivative: " << curvetureDerivative_ << std::endl;

  L_ = A_.row(5+nbConstraints_).head(6);

  lambda_n = 100;
  int lambda_max = 20;
  int lambda_min = 4;

  residualNorms.reserve(lambda_n);
  smoothnessNorms.reserve(lambda_n);
  sampledLambdas.reserve(lambda_n);
  curv.reserve(lambda_n);

  std::vector<double> tryLambdas;
  tryLambdas.reserve(lambda_n);

  double h = std::pow(lambda_max / lambda_min,  1. / (lambda_n - 1));

  for (size_t i = 0; i < lambda_n; i++)
  {
    double lambda = lambda_min * std::pow(h, i);

    tryLambdas.push_back(lambda);
  }
  


  //std::cout << "[" << i + 1 << "/" << lambda_n << "]" << std::endl;
  //std::cout << "lambda = " << lambda << std::endl;
  //std::cout << "--------" << std::endl;

  /**
   * Loop over regularization parameter values
   */

  // Generate the lambdas to get data for the L-curve
  //auto tryLambdas = linspace(lambda_min, lambda_max, lambda_n);

  for (auto lambda : tryLambdas){
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
    /*curve
    std::cout << "############" << std::endl;
    std::cout << "Lambda: " << lambda << std::endl;
    std::cout << "Norm of the eq: " << evalEqNorm2() << std::endl;
    std::cout << "Norm of regularization: " << evalSmoothnessConstraintNorm2() << std::endl;
    std::cout << "############" << std::endl;*/
    
    //std::cout << "isEqNorm2 inf: " << std::isinf(evalEqNorm2()) << std::endl;
    //std::cout << "smoothness inf: " << std::isinf(evalSmoothnessConstraintNorm2()) << std::endl;
    residualNorms.push_back(evalEqNorm2());
    smoothnessNorms.push_back(evalSmoothnessConstraintNorm2());
    //std::cout << "x_lambda solution: " << x_.transpose() << std::endl;
    //std::cout << "L_: " << L_.template cast<double>() << std::endl;


    sampledLambdas.push_back(lambda);
  }

  // We got the data now we need to fit a spline to it.
  //SPLINTER::DataTable samples;

  // Convert to log
  std::vector<double> logresidualNorms;
  logresidualNorms.reserve(residualNorms.size());
  for (auto residualNorm : residualNorms){

    if (residualNorm == 0.0)
    {
      // A dirty hack to avoid log10(0) = -inf
      continue;
    }

    //logresidualNorms.push_back(std::log10(1 / residualNorm));
    logresidualNorms.push_back(std::log10(residualNorm));
  }

  std::vector<double> logsmoothnessNorms;
  logsmoothnessNorms.reserve(smoothnessNorms.size());
  for (auto smoothnessNorm : smoothnessNorms){
    if (smoothnessNorm == 0.0)
    {
      // A dirty hack to avoid log10(0) = -inf
      continue;
    }
    
    //logsmoothnessNorms.push_back(std::log10(1 / smoothnessNorm));
    logsmoothnessNorms.push_back(std::log10(smoothnessNorm));
    //std::cout << "Value logsmoothness: " << std::log10(smoothnessNorm) << std::endl;
  }

  //auto table_y1 = SPLINTER::DataTable();
  auto dataTableFor_lambda_residual = SPLINTER::DataTable();
  auto dataTableFor_lambda_smoothness = SPLINTER::DataTable();

  //auto dataTableFor_residual_versus_smoothness = SPLINTER::DataTable();

  for(int j = 0; j < smoothnessNorms.size(); j++){
      // Store sample
      //samples.add_sample(tryLambdas[j],logsmoothnessNorms[j], logresidualNorms[j]);
      //table_y1.add_sample(tryLambdas[j],{logresidualNorms[j], logsmoothnessNorms[j]});
      dataTableFor_lambda_residual.add_sample(tryLambdas[j], logresidualNorms[j]);
      dataTableFor_lambda_smoothness.add_sample(tryLambdas[j], logsmoothnessNorms[j]);
      //dataTableFor_residual_versus_smoothness.add_sample(logresidualNorms[j], logsmoothnessNorms[j]);
  }

  //std::cout << "Converted to log" << std::endl;

  //const auto [min, max] = std::minmax_element(begin(logresidualNorms), end(logresidualNorms));

  //std::cout << "Found min max" << std::endl;
  //std::cout << "min: " << *min << std::endl;
  //std::cout << "max: " << *max << std::endl;

  const unsigned int y_dim = 1;
  const std::vector<unsigned int> degrees = {3};
  //const std::vector<std::vector<double>> knots = { {-5.0, -5.0, -5.0, -5.0, 10.0, 10.0, 10.0, 10.0} };
  //const std::vector<std::vector<double>> knots = { {*min - 1.0, *min - 1.0, *min - 1.0, *min - 1.0, *max + 1.0, *max + 1.0, *max + 1.0, *max + 1.0} };
  //double maxx = *max;
  //double minn = *min;

  //std::cout << "maxx: " << maxx << std::endl;
  //std::cout << "minn: " << minn << std::endl;

  //const std::vector<std::vector<double>> knots = {linspace(lambda_min - (10), lambda_max + (10), 2000)};
  //const std::vector<std::vector<double>> knots = {linspace(maxx, minn, 2000)};

 //double stepsize = ( (maxx + 0.1) - (minn - 0.1) ) / (2000);
  //SPLINTER::BSpline bspline3(degrees, knots, 2);
  //SPLINTER::BSpline bspline3 = bspline_interpolator(table_y1, 3);
  SPLINTER::BSpline bspline3_lambda_residual = bspline_interpolator(dataTableFor_lambda_residual, 3);
  SPLINTER::BSpline bspline3_lambda_smoothness = bspline_interpolator(dataTableFor_lambda_smoothness, 3);

  //SPLINTER::BSpline bspline3 = bspline_smoother(table_y1, 3, SPLINTER::BSpline::Smoothing::PSPLINE, 0.03);
  //SPLINTER::BSpline bspline3_lambda_residual = bspline_smoother(dataTableFor_lambda_residual, 3, SPLINTER::BSpline::Smoothing::PSPLINE, 0.03);
  //SPLINTER::BSpline bspline3_lambda_smoothness = bspline_smoother(dataTableFor_lambda_smoothness, 3, SPLINTER::BSpline::Smoothing::PSPLINE, 0.03);
  //SPLINTER::BSpline bspline3_residual_versus_smoothness = bspline_smoother(dataTableFor_residual_versus_smoothness, 3, SPLINTER::BSpline::Smoothing::PSPLINE, 0.03);
  //SPLINTER::BSpline bspline3_residual_versus_smoothness = bspline_interpolator(dataTableFor_residual_versus_smoothness, 3);
  //SPLINTER::BSpline bspline3_residual_versus_smoothness(degrees, knots, 1);

  bspline3_lambda_residual.fit(dataTableFor_lambda_residual);
  bspline3_lambda_smoothness.fit(dataTableFor_lambda_smoothness);
  //bspline3_residual_versus_smoothness.fit(dataTableFor_residual_versus_smoothness);
  //bspline3.fit(table_y1);

  // Constructed thespline

  //SPLINTER::DenseVector x(1);
  // Doesn't necessarily have to be sampled in the same size as the original data ?
  //auto x0_vec = linspace(minn + 5 * stepsize, maxx - 5 * stepsize, 2000);
  //auto x0_vec = linspace(lambda_min+1, lambda_max-1, lambda_n-2); // dont change lambda_n


  std::vector<double> x0_vec;
  x0_vec.reserve(lambda_n);

  double h2 = std::pow((lambda_max-1) / (lambda_min+1),  1. / (lambda_n - 3));

  for (size_t i = 0; i < lambda_n; i++)
  {
    double lambda = lambda_min * std::pow(h2, i);

    x0_vec.push_back(lambda);
  }


  //auto x0_vec = linspace(0., maxx - 0.1, 2000);

  //auto x0_vec =logresidualNorms;

  evalLambda.reserve(lambda_n);
  evalLambda = x0_vec;

  //auto x0_vec = tryLambdas;
  //std::vector<double> y0_vec;
  //y0_vec.reserve(x0_vec.size());

  std::vector<double> curvatureTry;
  curvatureTry.reserve(lambda_n);

  std::vector<double> splineEval_residual;
  splineEval_residual.reserve(lambda_n);

  std::vector<double> splineEval_smothness;
  splineEval_smothness.reserve(lambda_n);

  std::vector<double> jacob_residual_vector;
  jacob_residual_vector.reserve(lambda_n);

  std::vector<double> jacob_smoothness_vector;
  jacob_smoothness_vector.reserve(lambda_n);

  std::vector<double> hessian_residual_vector;
  hessian_residual_vector.reserve(lambda_n);

  std::vector<double> hessian_smoothness_vector;
  hessian_smoothness_vector.reserve(lambda_n);

  for (auto x0 : x0_vec)
  {
    //std::cout << "x0: " << x0 << std::endl;
    //y0_vec.push_back(bspline3.eval(std::vector<double>({x0})).at(0));

    //auto jacob = bspline3.eval_jacobian(std::vector<double>({x0}));
    //auto hessian = bspline3.eval_hessian(std::vector<double>({x0}));

    auto jacob_residual = bspline3_lambda_residual.eval_jacobian(std::vector<double>({x0}));
    auto jacob_smoothness = bspline3_lambda_smoothness.eval_jacobian(std::vector<double>({x0}));

    auto hessian_residual = bspline3_lambda_residual.eval_hessian(std::vector<double>({x0}));
    auto hessian_smoothness = bspline3_lambda_smoothness.eval_hessian(std::vector<double>({x0}));

    //auto jacob_dual = bspline3_residual_versus_smoothness.eval_jacobian(std::vector<double>({x0}));
    //auto hessian_dual = bspline3_residual_versus_smoothness.eval_hessian(std::vector<double>({x0}));

    /*std::cout << "hessian.size(): " << hessian.size() << std::endl;
    std::cout << "hessian.at(0).size(): " << hessian.at(0).size() << std::endl;
    std::cout << "hessian.at(1).size(): " << hessian.at(1).size() << std::endl;
    std::cout << "hessian.at(0).at(0).size(): " << hessian.at(0).at(0).size() << std::endl;
    std::cout << "hessian.at(1).at(0).size(): " << hessian.at(1).at(0).size() << std::endl;


    std::cout << "jacob.size(): " << jacob.size() << std::endl;
    std::cout << "jacob.at(0).size(): " << jacob.at(0).size() << std::endl;
    std::cout << "jacob.at(1).size(): " << jacob.at(1).size() << std::endl;*/
    
    double residual_firstDerivative = jacob_residual.at(0).at(0); // Change of the smoothness norm relative to lambda
    double smoothness_firstDerivative = jacob_smoothness.at(0).at(0); // Change of the smoothness norm relative to lambda

    double residual_secondDerivative = hessian_residual.at(0).at(0).at(0); // Change of the residual norm relative to lambda
    double smoothness_secondDerivative = hessian_smoothness.at(0).at(0).at(0); // Change of the residual norm relative to lambda

    //double dual_firstDerivative = jacob_dual.at(0).at(0); // Change of the smoothness norm relative to lambda
    //double dual_secondDerivative = hessian_dual.at(0).at(0).at(0); // Change of the residual norm relative to lambda

    //double Ezcurv = std::fabs(dual_secondDerivative)/std::pow(1.0 + dual_firstDerivative * dual_firstDerivative, 1.5);

    //double curv2 = std::fabs(residual_firstDerivative * smoothness_secondDerivative - residual_secondDerivative * smoothness_firstDerivative)/std::pow(residual_firstDerivative * residual_firstDerivative + smoothness_firstDerivative * smoothness_firstDerivative, 1.5);

    auto dividor = std::divides<void>{};

    auto ds_dr = smoothness_firstDerivative / residual_firstDerivative;

    auto d2s_dr2 = smoothness_secondDerivative / residual_secondDerivative;

    //double curv = dividor(std::fabs(d2s_dr2) , std::pow(1.0 + ds_dr * ds_dr, 1.5));

    //double curv = std::fabs(d2s_dr2);

    double curv = dividor(std::abs((residual_firstDerivative * smoothness_secondDerivative) - (residual_secondDerivative * smoothness_firstDerivative)), std::pow( std::pow(residual_firstDerivative,2) + std::pow(smoothness_firstDerivative,2), 1.5));

    // FIRST Y IS REGULAR RESIDUAL
    // SECOND Y IS SMOOTHNESS

    //double p_1 = jacob.at(0).at(0); // Change of the smoothness norm relative to lambda
    //double p_2 = hessian.at(0).at(0).at(0); // Change of the smoothness norm relative to lambda

    //double nu_1 = jacob.at(1).at(0); // Change of the residual norm relative to lambda
    //double nu_2 = hessian.at(1).at(0).at(0); // Change of the residual norm relative to lambda

    //double p_cur = std::fabs(p_2) / std::pow(1 + p_1 * p_1, 1.5);
    //double nu_cur = std::fabs(nu_2) / std::pow(1 + nu_1 * nu_1, 1.5);

    //double curv = p_cur + nu_cur;

    // Not baaaad, getting interesting values
    //double curv = std::fabs(p_1 * nu_2 - p_2 * nu_1)/std::pow(p_1 * p_1 + nu_1 * nu_1, 1.5);
    //double curv = std::fabs(nu_1 * p_2 - nu_2 * p_1)/std::pow(p_1 * p_1 + nu_1 * nu_1, 1.5);
    
    //double curv = std::abs(hessian.at(0).at(0).at(0)) / std::pow(1 + jacob.at(0).at(0) * jacob.at(0).at(0), 1.5);

    //std::cout << "curv: " << curv << std::endl;
    //std::cout << "residual_firstDerivative: " << residual_firstDerivative << std::endl;
    //std::cout << "smoothness_firstDerivative: " << smoothness_firstDerivative << std::endl;

    curvatureTry.push_back(curv);

    //splineEval_x.push_back(residual_firstDerivative);
    //splineEval_y.push_back(smoothness_firstDerivative);

    jacob_residual_vector.push_back(residual_firstDerivative);
    jacob_smoothness_vector.push_back(smoothness_firstDerivative);
    hessian_residual_vector.push_back(residual_secondDerivative);
    hessian_smoothness_vector.push_back(smoothness_secondDerivative);

    splineEval_residual.push_back(bspline3_lambda_residual.eval(std::vector<double>({x0})).at(0));
    splineEval_smothness.push_back(bspline3_lambda_smoothness.eval(std::vector<double>({x0})).at(0));
  }

  //https://github.com/PJLab-ADG/SensorsCalibration/blob/826f6fc290e7a33f4e1e1c1a47cf5ad6fa8d79e1/SensorX2car/lidar2car/src/yawcalib/yawCalib.cpp#L61

  /*curvatureTry.erase(std::remove_if(
      curvatureTry.begin(), curvatureTry.end(),
      [](const int& x) { 
          return x > 10000; // put your condition here
      }), curvatureTry.end()); */

  cpp_curveture.reserve(curvatureTry.size());
  cpp_curveture = curvatureTry;
  cpp_curveture_x_axis.reserve(lambda_n);
  cpp_curveture_x_axis = x0_vec;

  splineEval_residual_.reserve(lambda_n);
  splineEval_residual_ = splineEval_residual;

  splineEval_smothness_.reserve(lambda_n);
  splineEval_smothness_ = splineEval_smothness;

  jacob_smoothness_vector_.reserve(lambda_n);
  jacob_smoothness_vector_ = jacob_smoothness_vector;

  hessian_smoothness_vector_.reserve(lambda_n);
  hessian_smoothness_vector_ = hessian_smoothness_vector;

  jacob_residual_vector_.reserve(lambda_n);
  jacob_residual_vector_ = jacob_residual_vector;

  hessian_residual_vector_.reserve(lambda_n);
  hessian_residual_vector_ = hessian_residual_vector;


  /*std::vector<double> sums;

  sums.reserve(lambda_n);

  for (size_t i = 0; i < lambda_n; i++)
  {
    //sums.push_back(std::sqrt(std::pow(jacob_smoothness_vector_[i], 2) + std::pow( jacob_residual_vector_[i], 2)));

    sums.push_back(std::hypot(jacob_smoothness_vector_[i], jacob_residual_vector_[i]));
    std::cout << "Sum: " << sums[i] << std::endl;
  }

  const auto [min_sum, max_sum] = std::minmax_element(begin(sums), end(sums));

  auto munElementIndex = std::distance(sums.begin(), min_sum);*/


  //std::cout << "Jacob, hessian and curvature calculated" << std::endl;

  // Find the index of the maximum curvature.

  const auto [min_curve, max_curve] = std::minmax_element(begin(curvatureTry), end(curvatureTry));

  auto maxElementIndex = std::distance(curvatureTry.begin(), max_curve);

  std::cout << "Index of Max Curvature: " << maxElementIndex << std::endl;
  std::cout << "Maximum Curvature: " << *max_curve << std::endl;
  std::cout << "Minimum Curvature: " << *min_curve << std::endl;
  std::cout << "The lambda corresponding to max curvature: " << x0_vec[maxElementIndex] << std::endl;
  //std::cout << "Maximum Curvature using index: " << curvatureTry[maxElementIndex] << std::endl;

  optimalRegularizationWeight_ = x0_vec[maxElementIndex];

  //
  ////////auto iteratorToTheClosestValueInLogSmoothness = closest(logsmoothnessNorms, x0_vec[maxElementIndex]);

  // Use the found index to identify the closest value in the original x axis.
  /*auto iteratorToTheClosestValueInLogSmoothness = std::lower_bound(logsmoothnessNorms.begin(), logsmoothnessNorms.end(), x0_vec[maxElementIndex]); 

  // Find the index of the closest value in the original x axis.
  auto indexOf = std::distance(logsmoothnessNorms.begin(), iteratorToTheClosestValueInLogSmoothness);

  std::cout << "Index of lambda: " << indexOf << std::endl;
  std::cout << "Value of lambda: " << sampledLambdas[indexOf] << std::endl;

  optimalRegularizationWeight_ = sampledLambdas[indexOf]; */

}

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

  return evalLambda;
  //return sampledLambdas;
}

std::vector<double> LcurveOptimizer::getResiduals(){

  //return splineEval_x_;
  //return cpp_curveture;
  return residualNorms;
}

double LcurveOptimizer::getOptimalRegularizationWeight(){

  return optimalRegularizationWeight_;
}

std::vector<double> LcurveOptimizer::getRegNorms(){

  //return cpp_curveture_x_axis;
  //return splineEval_y_;
  return smoothnessNorms;
}

//////////////////////////

std::vector<double> LcurveOptimizer::getSmothnessJacob(){

  return jacob_smoothness_vector_;
}


std::vector<double> LcurveOptimizer::getSmothnessHessian(){

  return hessian_smoothness_vector_;
}

//////////////////////////
std::vector<double> LcurveOptimizer::getResidualJacob(){

  return jacob_residual_vector_;
}

std::vector<double> LcurveOptimizer::getResidualHessian(){

  return hessian_residual_vector_;
}

//////////////////////////
std::vector<double> LcurveOptimizer::getEvalSmothness(){

  return splineEval_smothness_;
}

std::vector<double> LcurveOptimizer::getEvalResidual(){

  return splineEval_residual_;
}

int LcurveOptimizer::getNumberOfSamples() {
  return lambda_n;
}

double LcurveOptimizer::evalEqNorm2() {
  Eigen::VectorXd dv = A_.template cast<double>() * x_ - b_.template cast<double>();
  return dv.stableNorm();
}

double LcurveOptimizer::evalSmoothnessConstraintNorm2() {
  //return x_.dot(L_.template cast<double>());
  //Eigen::Matrix<float, 1, 6> inter = L_.topLeftCorner(1, 6);
  return (L_.template cast<double>() * x_).stableNorm();
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