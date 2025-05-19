/*
 * Mapper.hpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#pragma once

#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Registration.h>
#include <Eigen/Geometry>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/SubmapCollection.hpp"
#include "open3d_slam/TransformInterpolationBuffer.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/time.hpp"

#include <nav_msgs/Path.h>

#include <algorithm>
#include <execution>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/factors/icp_factor.hpp>
#include <small_gicp/factors/plane_icp_factor.hpp>
#include <small_gicp/factors/robust_kernel.hpp>
#include <small_gicp/factors/symmetric_plane_icp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/reduction_omp_trimmed.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include "open3d_conversions/usings.hpp"

namespace o3d_slam {

class ScanToMapRegistration;

class Mapper {
 public:
  using PointCloud = open3d::geometry::PointCloud;

  Mapper(const TransformInterpolationBuffer& odomToRangeSensorBuffer, std::shared_ptr<SubmapCollection> submaps);
  ~Mapper() = default;

  void setParameters(const MapperParameters& p);
  void setMapToRangeSensor(const Transform& t);
  void setMapToRangeSensorInitial(const Transform& t);

  const Submap& getActiveSubmap() const;
  const SubmapCollection& getSubmaps() const;
  SubmapCollection* getSubmapsPtr();
  PointCloud getAssembledMapPointCloud() const;
  PointCloud getAssembledMapPointCloudVisualization() const;
  MapperParameters* getParametersPtr();
  Transform getMapToOdom(const Time& timestamp) const;
  Transform getMapToRangeSensor(const Time& timestamp) const;
  Transform getRegistrationBestGuess(const Time& timestamp) const;
  bool isRegistrationBestGuessBufferEmpty() const;
  const TransformInterpolationBuffer& getMapToRangeSensorBuffer() const;
  const PointCloud& getPreprocessedScan() const;
  const ScanToMapRegistration& getScanToMapRegistration() const;

  void loopClosureUpdate(const Transform& loopClosureCorrection);
  bool hasProcessedMeasurements() const;
  bool addRangeMeasurement(const PointCloud& cloud, const Time& timestamp);

  void setExternalOdometryFrameToCloudFrameCalibration(const Eigen::Isometry3d& transform);
  bool isExternalOdometryFrameToCloudFrameCalibrationSet();

  void copyOrEstimateNormals(const open3d::geometry::PointCloud& src, small_gicp::PointCloud& dst,
                             small_gicp::KdTree<small_gicp::PointCloud>& dst_tree, int normal_knn = 10, int num_threads = 0);

  // This is re-initialized in the constructor as well as by a setter.
  Transform calibration_ = Transform::Identity();
  Transform lastReferenceInitializationPose_ = Transform::Identity();
  bool isCalibrationSet_ = false;

  nav_msgs::Path trackedPath_;
  nav_msgs::Path bestGuessPath_;
  bool isNewValueSetMapper_ = false;
  bool isInitialTransformSet_ = false;

  using RegistrationType = small_gicp::Registration<
      small_gicp::RobustFactor<small_gicp::Cauchy, small_gicp::SymmetricPointToPlaneICPFactor>, small_gicp::ParallelReductionOMP,
      small_gicp::NullFactor,                          // Empty_Factor, Cauchy, Huber
      small_gicp::CompoundRejector,                    // CompoundRejector //DistanceRejector //NullRejector // ParallelReductionOMPTrimmed
      small_gicp::RobustLevenbergMarquardtOptimizer>;  // SymmetricPointToPlaneICPFactor //PointToPlaneICPFactor
                                                       // //HouseholderSolver LevenbergMarquardtOptimizer
                                                       // //RobustLevenbergMarquardtOptimizer //GaussNewtonOptimizer ICPFactor
                                                       // //NullRejector
                                                       // //Cauchy

  // using RegistrationType = small_gicp::Registration<
  //     small_gicp::RobustFactor<small_gicp::Cauchy, small_gicp::SymmetricPointToPlaneICPFactor>, small_gicp::ParallelReductionOMP,
  //     small_gicp::NullFactor,
  //     small_gicp::NullRejector,                        // DistanceRejector //
  //     small_gicp::RobustLevenbergMarquardtOptimizer>;  // SymmetricPointToPlaneICPFactor //RobustLevenbergMarquardtOptimizer //
  //                                                      // LevenbergMarquardtOptimizer // ICPFactor //NullRejector //Cauchy

  // // small_gicp::Registration<small_gicp::RobustFactor<small_gicp::Cauchy, small_gicp::GICPFactor>, small_gicp::ParallelReductionOMP>
  // //     small_registration_;

  RegistrationType small_registration_;

  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> source_tree_;

  std::shared_ptr<small_gicp::PointCloud> target_;
  std::shared_ptr<small_gicp::PointCloud> source_;
  // pointmatcher_ros::PmIcp icp_;

  Eigen::Vector3d c_t = Eigen::Vector3d::Zero();
  Eigen::Vector3d c_s = Eigen::Vector3d::Zero();

  Eigen::Vector3d computeCentroid(const small_gicp::PointCloud& pc);
  void translatePointCloud(small_gicp::PointCloud& pc, const Eigen::Vector3d& t);

 private:
  void update(const MapperParameters& p);
  void checkTransformChainingAndPrintResult(bool isCheckTransformChainingAndPrintResult) const;

  Time lastMeasurementTimestamp_;
  Time lastReferenceInitializationTimestamp_;
  Time initTime_;
  Timer testmapperOnlyTimer_;
  Timer referenceInitTimer_;
  Timer auxilaryTimer_;
  Timer scanInsertionTimer_;

  Transform mapToRangeSensor_ = Transform::Identity();
  Transform mapToRangeSensorPrev_ = Transform::Identity();
  Transform mapToRangeSensorLastScanInsertion_ = Transform::Identity();
  Transform bestGuessMemory_ = Transform::Identity();
  Transform odometryMotionMemory_ = Transform::Identity();

  MapperParameters params_;
  mutable std::mutex mapManipulationMutex_;
  std::shared_ptr<SubmapCollection> submaps_;
  const TransformInterpolationBuffer& odomToRangeSensorBuffer_;
  TransformInterpolationBuffer mapToRangeSensorBuffer_;
  TransformInterpolationBuffer bestGuessBuffer_;
  open3d::geometry::PointCloud preProcessedScan_;

  bool isIgnoreOdometryPrediction_ = false;
  bool firstRefinement_ = true;

  std::shared_ptr<ScanToMapRegistration> scan2MapReg_;
  // std::shared_ptr<open3d_conversions::PmPointCloudFilters> pmPointCloudFilter_;
};

} /* namespace o3d_slam */
