/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#pragma once

// C++
#include <Eigen/Eigen>
// Workspace
#include "open3d_slam_ros/Color.hpp"
#include "open3d_slam_ros/Gnss.hpp"

namespace o3d_slam {

class GnssHandler {
 public:
  GnssHandler();

  // Methods
  void initHandler(const Eigen::Vector3d& accumulatedLeftCoordinates, const Eigen::Vector3d& accumulatedRightCoordinates);
  void initHandler(const Eigen::Vector3d& accumulatedCoordinates);
  void initHandler(const double& initYaw);

  void convertNavSatToPositions(const Eigen::Vector3d& leftGnssCoordinate, const Eigen::Vector3d& rightGnssCoordinate,
                                Eigen::Vector3d& leftPosition, Eigen::Vector3d& rightPosition);
  void convertNavSatToPosition(const Eigen::Vector3d& gnssCoordinate, Eigen::Vector3d& position);
  double computeYaw(const Eigen::Vector3d& gnssPos1, const Eigen::Vector3d& gnssPos2);

  // Flags
  bool usingGnssReferenceFlag = false;

  // Setters
  void setInitYaw(const double initYaw) { initYaw_ = initYaw; }
  void setGnssReferenceLatitude(const double gnssReferenceLatitude) { gnssReferenceLatitude_ = gnssReferenceLatitude; }
  void setGnssReferenceLongitude(const double gnssReferenceLongitude) { gnssReferenceLongitude_ = gnssReferenceLongitude; }
  void setGnssReferenceAltitude(const double gnssReferenceAltitude) { gnssReferenceAltitude_ = gnssReferenceAltitude; }
  void setGnssReferenceHeading(const double gnssReferenceHeading) { gnssReferenceHeading_ = gnssReferenceHeading; }

  // Getters.
  double getInitYaw();
  Gnss getGnssSensor() { return gnssSensor_; }

 private:
  // Member methods
  Eigen::Vector3d computeHeading_(const Eigen::Vector3d& gnssPos1, const Eigen::Vector3d& gnssPos2);
  //// Compute yaw from the heading vector
  static double computeYawFromHeadingVector_(const Eigen::Vector3d& headingVector);

  // Member variables
  Gnss gnssSensor_;
  Eigen::Vector3d W_t_W_GnssL0_;
  double globalAttitudeYaw_;

  // Reference Parameters
  double initYaw_;
  double gnssReferenceLatitude_;
  double gnssReferenceLongitude_;
  double gnssReferenceAltitude_;
  double gnssReferenceHeading_;
};

}  // namespace o3d_slam
