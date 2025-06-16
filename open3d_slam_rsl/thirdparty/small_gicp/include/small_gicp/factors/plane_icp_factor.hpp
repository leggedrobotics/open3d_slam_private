// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <small_gicp/util/lie.hpp>
#include <small_gicp/ann/traits.hpp>
#include <small_gicp/points/traits.hpp>

namespace small_gicp {

/// @brief Point-to-plane per-point error factor.
struct PointToPlaneICPFactor {
  struct Setting {};

  PointToPlaneICPFactor(const Setting& setting = Setting()) : target_index(std::numeric_limits<size_t>::max()), source_index(std::numeric_limits<size_t>::max()) {}

  template <typename TargetPointCloud, typename SourcePointCloud, typename TargetTree, typename CorrespondenceRejector>
  bool linearize(
    const TargetPointCloud& target,
    const SourcePointCloud& source,
    const TargetTree& target_tree,
    const Eigen::Isometry3d& T,
    size_t source_index,
    const CorrespondenceRejector& rejector,
    Eigen::Matrix<double, 6, 6>* H,
    Eigen::Matrix<double, 6, 1>* b,
    double* e) {
    //
    this->source_index = source_index;
    this->target_index = std::numeric_limits<size_t>::max();

    const Eigen::Vector4d transed_source_pt = T * traits::point(source, source_index);

    constexpr size_t k_neighbors = 11;

    std::array<size_t, k_neighbors>  k_indices;
    std::array<double, k_neighbors>  k_sq_dists;

    if (traits::knn_search(target_tree, transed_source_pt, k_neighbors, k_indices.data(), k_sq_dists.data()) != k_neighbors) {
        return false;
    }

    // Initialize accumulators
    H->setZero();
    b->setZero();
    *e = 0.0;

    bool found = false;

    for (size_t i = 0; i < k_neighbors; ++i) {
        size_t k_index = k_indices[i];
        double sq_dist = k_sq_dists[i];
        if (rejector(target, source, T, k_index, source_index, sq_dist)) continue;

        found = true;

        const auto& target_normal = traits::normal(target, k_index);

        const Eigen::Vector4d residual = traits::point(target, k_index) - transed_source_pt;
        const Eigen::Vector4d err = target_normal.array() * residual.array();

        Eigen::Matrix<double, 4, 6> J = Eigen::Matrix<double, 4, 6>::Zero();
        J.block<3, 3>(0, 0) = target_normal.template head<3>().asDiagonal() * T.linear() * skew(traits::point(source, source_index).template head<3>());
        J.block<3, 3>(0, 3) = target_normal.template head<3>().asDiagonal() * (-T.linear());

        *H += J.transpose() * J;
        *b += J.transpose() * err;
        *e += 0.5 * err.squaredNorm();
    }

    return found;
  }

  template <typename TargetPointCloud, typename SourcePointCloud>
  double error(const TargetPointCloud& target, const SourcePointCloud& source, const Eigen::Isometry3d& T) const {
    if (target_index == std::numeric_limits<size_t>::max()) {
      return 0.0;
    }

    const Eigen::Vector4d transed_source_pt = T * traits::point(source, source_index);
    const Eigen::Vector4d residual = traits::point(target, target_index) - transed_source_pt;

    const auto& normal = traits::normal(target, target_index);

    double proj = normal.dot(residual);
    return 0.5 * proj * proj;

    // const Eigen::Vector4d error = traits::normal(target, target_index).array() * residual.array();
    // return 0.5 * error.squaredNorm();
  }

  bool inlier() const { return target_index != std::numeric_limits<size_t>::max(); }

  size_t target_index;
  size_t source_index;
};
}  // namespace small_gicp
