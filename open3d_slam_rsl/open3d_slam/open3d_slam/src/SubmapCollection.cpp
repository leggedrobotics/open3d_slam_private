/*
 * SubmapCollection.cpp
 *
 *  Created on: Nov 10, 2021
 *      Author: jelavice
 */

#include "open3d_slam/SubmapCollection.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/constraint_builders.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/magic.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/typedefs.hpp"

#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/pipelines/registration/Registration.h>

#include <algorithm>
#include <numeric>
#include <set>
#include <thread>
#include <utility>

#include "open3d_slam/ProfilerScopeGuard.hpp"

namespace o3d_slam {

SubmapCollection::SubmapCollection() {
  submaps_.reserve(500);
  lastVisitPosition_.resize(submaps_.size(), Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));

  overlapScansBuffer_.set_size_limit(100);
}

void SubmapCollection::setMapToRangeSensor(const Transform& T) {
  mapToRangeSensor_ = T;
}

bool SubmapCollection::isEmpty() const {
  return submaps_.empty();
}

Submap* SubmapCollection::getSubmapPtr(SubmapId idx) {
  return &(submaps_.at(idx));
}

const Submap& SubmapCollection::getSubmap(SubmapId idx) const {
  return submaps_.at(idx);
}
size_t SubmapCollection::getNumSubmaps() const {
  return submaps_.size();
}

SubmapCollection::TimestampedSubmapIds SubmapCollection::popFinishedSubmapIds() {
  return finishedSubmapsIdxs_.popAllElements();
}

SubmapCollection::TimestampedSubmapIds SubmapCollection::popLoopClosureCandidates() {
  return loopClosureCandidatesIdxs_.popAllElements();
}

size_t SubmapCollection::numFinishedSubmaps() const {
  return finishedSubmapsIdxs_.size();
}

size_t SubmapCollection::numLoopClosureCandidates() const {
  return loopClosureCandidatesIdxs_.size();
}

size_t SubmapCollection::getTotalNumPoints() const {
  const int nSubmaps = submaps_.size();
  return std::accumulate(submaps_.begin(), submaps_.end(), 0,
                         [](size_t sum, const Submap& s) { return sum + s.getMapPointCloud().points_.size(); });
}

void SubmapCollection::updateAdjacencyMatrix(const Constraints& loopClosureConstraints) {
  for (const auto& c : loopClosureConstraints) {
    adjacencyMatrix_.addEdge(c.sourceSubmapIdx_, c.targetSubmapIdx_);
    adjacencyMatrix_.markAsLoopClosureSubmap(c.sourceSubmapIdx_);
    adjacencyMatrix_.markAsLoopClosureSubmap(c.targetSubmapIdx_);
  }
}

void SubmapCollection::addScanToBuffer(const PointCloud& scan, const Transform& mapToRangeSensor, const Time& timestamp) {
  overlapScansBuffer_.push(ScanTimeTransform{scan, timestamp, mapToRangeSensor});
}

void SubmapCollection::insertBufferedScans(Submap* submap) {
  while (!overlapScansBuffer_.empty()) {
    auto scan = overlapScansBuffer_.pop();
    submap->insertScan(scan.cloud_, scan.cloud_, scan.mapToRangeSensor_, scan.timestamp_, false);
  }
}

void SubmapCollection::updateActiveSubmap(const Transform& mapToRangeSensor, const PointCloud& scan) {
  /* ---- forced creation --------------------------------------------------- */
  if (isForceNewSubmapCreation_) {
    std::cout << "\033[1;48;5;208m\033[1;30m"
              << "==================== SUBMAP CREATION FORCED ====================\n"
              << "   A new submap is being created due to force trigger!\n"
              << "===============================================================\033[0m\n";

    createNewSubmap(mapToRangeSensor_);
    isForceNewSubmapCreation_ = false;
    return;
  }
  if (params_.isUseInitialMap_) return;  // localisation‑only mode

  /* ---- constants --------------------------------------------------------- */
  const Eigen::Vector3d robot = mapToRangeSensor_.translation();
  const size_t N = submaps_.size();
  const size_t cur_idx = activeSubmapIdx_;
  const Eigen::Vector3d cur_ctr = submaps_[cur_idx].getMapToSubmapCenter();
  const double d_cur = (robot - cur_ctr).norm();

  /* ---- single‑submap case ------------------------------------------------ */
  if (N == 1) {
    if (d_cur > params_.submaps_.radius_) {
      std::cout << "\033[1;48;5;208m\033[1;30m"
                << "==================== NEW SUBMAP CREATED ====================\n"
                << "   Robot is outside the submap radius (1 submap case.).\n"
                << "   Current idx: " << cur_idx << ", d_cur=" << d_cur << "\n"
                << "============================================================\033[0m\n";
      createNewSubmap(mapToRangeSensor_);
    }
    return;
  }

  /* ---- nearest distance (current included) ------------------------------- */
  size_t nearest_idx = findClosestSubmap(mapToRangeSensor_, SIZE_MAX);
  const double d_near = (robot - submaps_[nearest_idx].getMapToSubmapCenter()).norm();

  /* ---- outside *all* radii? --------------------------------------------- */
  if (d_near > params_.submaps_.radius_) {
    std::cout << "\033[1;48;5;208m\033[1;30m"
              << "==================== NEW SUBMAP CREATED ====================\n"
              << "   Robot is outside every submap radius.\n"
              << "   Current idx: " << cur_idx << ", nearest idx: " << nearest_idx << "\n   d_near=" << d_near << ", d_cur=" << d_cur
              << "\n"
              << "============================================================\033[0m\n";
    createNewSubmap(mapToRangeSensor_);
    return;
  }

  /* ---- nearest is current – stay put ------------------------------------ */
  if (nearest_idx == cur_idx) return;

  /* ---- check adj. + consistency ----------------------------------------- */
  bool adj = adjacencyMatrix_.isAdjacent(submaps_[nearest_idx].getId(), submaps_[cur_idx].getId());
  bool ok = isSwitchingSubmapsConsistant(scan, nearest_idx, mapToRangeSensor);
  if (!adj || !ok) {
    if (d_cur > params_.submaps_.radius_) {
      createNewSubmap(mapToRangeSensor_);
      std::cout << "\033[1;48;5;208m\033[1;30m"
                << "==================== NEW SUBMAP CREATED ====================\n"
                << "   !adj || !ok.\n"
                << "   Current idx: " << cur_idx << ", nearest idx: " << nearest_idx << "\n   d_near=" << d_near << ", d_cur=" << d_cur
                << "\n"
                << "============================================================\033[0m\n";
    }
    return;
  }

  /* ---- switch only if moved >½ radius and closer to candidate ----------- */
  if (d_cur > 0.5 * params_.submaps_.radius_ && d_near < d_cur) {
    activeSubmapIdx_ = nearest_idx;
  }
}

std::vector<size_t> SubmapCollection::findKClosestSubmaps(const Transform& T, size_t k, size_t exclude) const {
  struct Pair {
    double d;
    size_t i;
  };
  std::vector<Pair> tmp;
  tmp.reserve(submaps_.size());
  Eigen::Vector3d p0 = T.translation();

  for (size_t i = 0; i < submaps_.size(); ++i)
    if (i != exclude) tmp.push_back({(p0 - submaps_[i].getMapToSubmapCenter()).norm(), i});

  std::partial_sort(tmp.begin(), tmp.begin() + std::min(k, tmp.size()), tmp.end(), [](const Pair& a, const Pair& b) { return a.d < b.d; });

  std::vector<size_t> out;
  for (size_t j = 0; j < k && j < tmp.size(); ++j) out.push_back(tmp[j].i);
  return out;
}

PointCloudPtr SubmapCollection::getCachedCompositeSubmapFromVoxel(const std::vector<size_t>& neighbor_idxs) const {
  std::lock_guard<std::mutex> lock(composite_cache_mutex_);
  // Detect if cache needs update
  if (!cached_neighbor_cloud_ || cached_active_submap_idx_ != activeSubmapIdx_ || cached_neighbor_idxs_ != neighbor_idxs) {
    // (Re)build and cache
    cached_neighbor_cloud_ = std::make_shared<PointCloud>(buildCompositeSubmapFromVoxel(neighbor_idxs));
    cached_active_submap_idx_ = activeSubmapIdx_;
    cached_neighbor_idxs_ = neighbor_idxs;
  }
  return cached_neighbor_cloud_;
}

PointCloud SubmapCollection::buildCompositeSubmapFromVoxel(const std::vector<size_t>& idxs) const {
  const size_t n = idxs.size();
  std::vector<std::vector<Eigen::Vector3d>> thread_points(n);
  std::vector<std::vector<Eigen::Vector3d>> thread_normals(n);

#pragma omp parallel for
  for (int i = 0; i < static_cast<int>(n); ++i) {
    const auto& submap = submaps_[idxs[i]];

    // If this is the active submap, always use the raw point cloud
    if (idxs[i] == activeSubmapIdx_) {
      const auto& pc = submap.getMapPointCloud();
      thread_points[i].assign(pc.points_.begin(), pc.points_.end());
      if (pc.HasNormals()) thread_normals[i].assign(pc.normals_.begin(), pc.normals_.end());
      continue;
    }

    // Otherwise, use the voxel map if it has points
    const auto& voxel_map = submap.getVoxelMap();
    size_t voxel_count = 0;
    for (const auto& kv : voxel_map.voxels_) {
      if (!kv.second.idxs_.empty()) ++voxel_count;
    }

    if (voxel_count > 0) {
      thread_points[i].reserve(voxel_count);
      thread_normals[i].reserve(voxel_count);

      for (const auto& kv : voxel_map.voxels_) {
        if (!kv.second.idxs_.empty()) {
          thread_points[i].push_back(getVoxelCenter(kv.first, voxel_map.getVoxelSize()));
          if (kv.second.num_aggregated_normals_ > 0) thread_normals[i].push_back(kv.second.getAggregatedNormal());
        }
      }
    } else {
      // Fallback: use raw point cloud
      const auto& pc = submap.getMapPointCloud();
      thread_points[i].assign(pc.points_.begin(), pc.points_.end());
      if (pc.HasNormals()) thread_normals[i].assign(pc.normals_.begin(), pc.normals_.end());
    }
  }

  // Compute output sizes
  size_t total_points = 0, total_normals = 0;
  for (size_t i = 0; i < n; ++i) {
    total_points += thread_points[i].size();
    total_normals += thread_normals[i].size();
  }

  PointCloud cloud;
  cloud.points_.resize(total_points);
  if (total_normals > 0) cloud.normals_.resize(total_normals);

  constexpr size_t PARALLEL_THRESHOLD = 20000;
  if (total_points > PARALLEL_THRESHOLD) {
    std::vector<size_t> point_offsets(n + 1, 0), normal_offsets(n + 1, 0);
    for (size_t i = 0; i < n; ++i) {
      point_offsets[i + 1] = point_offsets[i] + thread_points[i].size();
      normal_offsets[i + 1] = normal_offsets[i] + thread_normals[i].size();
    }

#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(n); ++i) {
      std::copy(thread_points[i].begin(), thread_points[i].end(), cloud.points_.begin() + point_offsets[i]);
      if (!thread_normals[i].empty())
        std::copy(thread_normals[i].begin(), thread_normals[i].end(), cloud.normals_.begin() + normal_offsets[i]);
    }
  } else {
    size_t point_pos = 0, normal_pos = 0;
    for (size_t i = 0; i < n; ++i) {
      for (auto& p : thread_points[i]) cloud.points_[point_pos++] = std::move(p);
      for (auto& nrm : thread_normals[i]) cloud.normals_[normal_pos++] = std::move(nrm);
    }
  }

  return cloud;
}

PointCloudPtr SubmapCollection::getCachedCompositeSubmapFromMulti(const std::vector<size_t>& all_idxs) const {
  std::lock_guard<std::mutex> lk(composite_cache_mutex_);

  const size_t active = activeSubmapIdx_;
  std::vector<size_t> static_idxs;
  static_idxs.reserve(all_idxs.size());
  for (size_t id : all_idxs)
    if (id != active) static_idxs.push_back(id);

  /* ---------- 1. (Re)build static neighbour patch if needed ----------- */
  if (!cached_static_cloud_ || cached_static_idxs_ != static_idxs) {
    cached_static_cloud_ = std::make_shared<PointCloud>(buildCompositeSubmapFromMulti(static_idxs));
    cached_static_point_count_ = cached_static_cloud_->points_.size();
    cached_static_idxs_ = static_idxs;
    // invalidate combined cache – forces rebuild below
    cached_combined_cloud_.reset();
  }

  /* ---------- 2. Always create the current active cloud --------------- */
  PointCloudPtr active_cloud = std::make_shared<PointCloud>(submaps_[active].getMapPointCloud());
  const size_t active_pts = active_cloud->points_.size();

  /* ---------- 3. Re-use / build combined patch ------------------------ */
  if (!cached_combined_cloud_ || cached_active_point_count_ != active_pts) {
    cached_combined_cloud_ = std::make_shared<PointCloud>();
    cached_combined_cloud_->points_.reserve(cached_static_point_count_ + active_pts);

    // copy neighbours (static) – one time until they change
    cached_combined_cloud_->points_ = cached_static_cloud_->points_;
    if (cached_static_cloud_->HasNormals()) cached_combined_cloud_->normals_ = cached_static_cloud_->normals_;

    // append active
    cached_combined_cloud_->points_.insert(cached_combined_cloud_->points_.end(), active_cloud->points_.begin(),
                                           active_cloud->points_.end());

    if (active_cloud->HasNormals()) {
      if (!cached_combined_cloud_->HasNormals())
        cached_combined_cloud_->normals_.resize(cached_static_cloud_->normals_.size(), Eigen::Vector3d::Zero());
      cached_combined_cloud_->normals_.insert(cached_combined_cloud_->normals_.end(), active_cloud->normals_.begin(),
                                              active_cloud->normals_.end());
    }
    cached_active_point_count_ = active_pts;
  }

  return cached_combined_cloud_;
}

PointCloud SubmapCollection::buildCompositeSubmapFromMulti(const std::vector<size_t>& idxs) const {
  // Extract per-submap in parallel
  const size_t n = idxs.size();
  std::vector<std::vector<Eigen::Vector3d>> thread_points(n);
  std::vector<std::vector<Eigen::Vector3d>> thread_normals(n);

#pragma omp parallel for
  for (int i = 0; i < static_cast<int>(n); ++i) {
    const auto& submap = submaps_[idxs[i]];
    const auto& pc = submap.getMapPointCloud();
    thread_points[i].assign(pc.points_.begin(), pc.points_.end());
    if (pc.HasNormals()) thread_normals[i].assign(pc.normals_.begin(), pc.normals_.end());
  }

  // Compute sizes
  size_t total_points = 0;
  size_t total_normals = 0;
  for (size_t i = 0; i < n; ++i) {
    total_points += thread_points[i].size();
    total_normals += thread_normals[i].size();
  }

  PointCloud cloud;
  cloud.points_.resize(total_points);
  if (total_normals > 0) cloud.normals_.resize(total_normals);

  // Parallel merge if big enough, else serial
  constexpr size_t PARALLEL_THRESHOLD = 20000;
  if (total_points > PARALLEL_THRESHOLD) {
    std::vector<size_t> point_offsets(n + 1, 0);
    std::vector<size_t> normal_offsets(n + 1, 0);
    for (size_t i = 0; i < n; ++i) {
      point_offsets[i + 1] = point_offsets[i] + thread_points[i].size();
      normal_offsets[i + 1] = normal_offsets[i] + thread_normals[i].size();
    }

#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(n); ++i) {
      // Copy points
      std::copy(thread_points[i].begin(), thread_points[i].end(), cloud.points_.begin() + point_offsets[i]);
      // Copy normals if present
      if (!thread_normals[i].empty()) {
        std::copy(thread_normals[i].begin(), thread_normals[i].end(), cloud.normals_.begin() + normal_offsets[i]);
      }
    }
  } else {
    // Serial merge for small clouds
    size_t point_pos = 0, normal_pos = 0;
    for (size_t i = 0; i < n; ++i) {
      for (auto& p : thread_points[i]) cloud.points_[point_pos++] = std::move(p);
      for (auto& nrm : thread_normals[i]) cloud.normals_[normal_pos++] = std::move(nrm);
    }
  }

  return cloud;
}

PointCloud SubmapCollection::buildCompositeSubmapSingle(const std::vector<size_t>& idxs, double voxel) const {
  PointCloud cloud;

  // Prepare per-thread vectors
  std::vector<std::vector<Eigen::Vector3d>> thread_points(idxs.size());
  std::vector<std::vector<Eigen::Vector3d>> thread_normals(idxs.size());

#pragma omp parallel for
  for (int i = 0; i < static_cast<int>(idxs.size()); ++i) {
    const auto& pc = submaps_[idxs[i]].getMapPointCloud();
    thread_points[i].assign(pc.points_.begin(), pc.points_.end());
    if (pc.HasNormals()) thread_normals[i].assign(pc.normals_.begin(), pc.normals_.end());
  }

  // Merge all into the output cloud (single thread)
  size_t total = 0;
  for (const auto& v : thread_points) total += v.size();
  cloud.points_.reserve(total);

  for (auto& v : thread_points) cloud.points_.insert(cloud.points_.end(), v.begin(), v.end());
  if (!thread_normals.empty() && !thread_normals[0].empty()) {
    for (auto& v : thread_normals) cloud.normals_.insert(cloud.normals_.end(), v.begin(), v.end());
  }

  // Downsample
  if (voxel > 0.0) cloud = *cloud.VoxelDownSample(voxel);
  return cloud;
}

void SubmapCollection::createNewSubmap(const Transform& mapToSubmap) {
  std::lock_guard<std::mutex> lk(consistency_cache_mtx_);

  const size_t id = submapId_++;
  const size_t pid = activeSubmapIdx_;

  Submap m(id, pid);
  m.setMapToSubmapOrigin(mapToSubmap);
  m.setSubmapCenter(mapToSubmap.translation());  // centre fixed at creation
  m.setParameters(params_);

  submaps_.emplace_back(std::move(m));
  consistency_cache_.emplace_back();
  lastVisitPosition_.push_back(mapToRangeSensor_.translation());

  activeSubmapIdx_ = submaps_.size() - 1;
  numScansMergedInActiveSubmap_ = 0;
}

size_t SubmapCollection::findClosestSubmap(const Transform& mapToRS, size_t exclude_idx) const {
  const Eigen::Vector3d robot = mapToRS.translation();
  size_t best = SIZE_MAX;
  double best_d = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < submaps_.size(); ++i) {
    if (i == exclude_idx) continue;
    double d = (robot - submaps_[i].getMapToSubmapCenter()).norm();
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

const Submap& SubmapCollection::getActiveSubmap() const {
  return submaps_.at(activeSubmapIdx_);
}

void SubmapCollection::forceNewSubmapCreation() {
  if (submaps_.empty()) {
    return;
  }
  isForceNewSubmapCreation_ = true;
  insertScan(PointCloud(), PointCloud(), mapToRangeSensor_, timestamp_);
  isForceNewSubmapCreation_ = false;
}

bool SubmapCollection::insertScan(const PointCloud& raw, const PointCloud& proc, const Transform& mapToRS, const Time& stamp) {
  ProfilerScopeGuard total("SubmapCollection::insertScan", "/tmp/slam_profile.csv", "submap=" + std::to_string(activeSubmapIdx_));

  mapToRangeSensor_ = mapToRS;
  timestamp_ = stamp;
  const size_t prevActiveSubmapIdx = activeSubmapIdx_;

  /* ---- first scan ------------------------------------------------------- */
  if (submaps_.empty()) {
    std::cout << "\033[1;48;5;208m\033[1;30m"
              << "==================== FIRST SUBMAP CREATED ====================\n"
              << "   First scan received. Creating first submap.\n"
              << "==============================================================\033[0m\n";
    createNewSubmap(mapToRangeSensor_);
    submaps_[activeSubmapIdx_].insertScan(raw, proc, mapToRS, stamp, true);
    // centre stays fixed → no recomputation here
    ++numScansMergedInActiveSubmap_;
    return true;
  }

  addScanToBuffer(proc, mapToRS, stamp);
  updateActiveSubmap(mapToRS, proc);

  const bool switched = (prevActiveSubmapIdx != activeSubmapIdx_);

  if (switched) {
    std::cout << "\033[1;44m\033[1;97m"
              << "==================== ACTIVE SUBMAP CHANGED ====================\n"
              << "   FROM SUBMAP ID: " << prevActiveSubmapIdx << "   TO SUBMAP ID: " << activeSubmapIdx_ << "\n"
              << "==============================================================\033[0m\n";

    /* finish previous ----------------------------------------------------- */
    {
      std::lock_guard<std::mutex> lck(featureComputationMutex_);
      submaps_[prevActiveSubmapIdx].insertScan(raw, proc, mapToRS, stamp, true);
    }
    submaps_[prevActiveSubmapIdx].computeSubmapCenter();  // recompute centre ON EXIT

    finishedSubmapsIdxs_.push({prevActiveSubmapIdx, stamp});
    numScansMergedInActiveSubmap_ = 0;

    /* add edge in adjacency ---------------------------------------------- */
    adjacencyMatrix_.addEdge(submaps_[prevActiveSubmapIdx].getId(), submaps_[activeSubmapIdx_].getId());

    insertBufferedScans(&submaps_[activeSubmapIdx_]);
  } else {
    submaps_[activeSubmapIdx_].insertScan(raw, proc, mapToRS, stamp, true);
  }

  ++numScansMergedInActiveSubmap_;
  return true;
}

void SubmapCollection::setParameters(const MapperParameters& p) {
  params_ = p;
  for (auto& submap : submaps_) {
    submap.setParameters(p);
  }
  placeRecognition_.setParameters(p);
  assert_gt<size_t>(params_.submaps_.numScansOverlap_, 0, "Num scan overlap has to be > 0");
  overlapScansBuffer_.set_size_limit(params_.submaps_.numScansOverlap_);
}

void SubmapCollection::computeFeatures(const TimestampedSubmapIds& finishedSubmapIds) {
  std::lock_guard<std::mutex> lck(featureComputationMutex_);
  isComputingFeatures_ = true;
  Timer timer("submap_finishing feature + constraint comp: ");

  auto featureComputation = [&]() {
    //		Timer t("feature computation");
    for (const auto& id : finishedSubmapIds) {
      // std::cout << "computing features for submap: " << id.submapId_ << std::endl;
      // std::cout << "submap size: " << submaps_.at(id.submapId_).getMapPointCloud().points_.size() << std::endl;
      submaps_.at(id.submapId_).computeFeatures();
      loopClosureCandidatesIdxs_.push(id);
    }
  };

  std::thread t(featureComputation);

  {
    //		Timer t("odometry_constraint_computation");
    computeOdometryConstraints(*this, finishedSubmapIds, &odometryConstraints_);
  }

  t.join();
  isComputingFeatures_ = false;
}

bool SubmapCollection::isComputingFeatures() const {
  return isComputingFeatures_;
}

const Constraints& SubmapCollection::getOdometryConstraints() const {
  return odometryConstraints_;
}

Constraints SubmapCollection::buildLoopClosureConstraints(const TimestampedSubmapIds& loopClosureCandidatesIdxs) {
  Constraints retVal;
  for (const auto& id : loopClosureCandidatesIdxs) {
    const auto constraints =
        placeRecognition_.buildLoopClosureConstraints(mapToRangeSensor_, *this, adjacencyMatrix_, id.submapId_, activeSubmapIdx_, id.time_);
    if (!constraints.empty()) {
      for (int i = 0; i < constraints.size(); ++i) {
        retVal.push_back(constraints.at(i));
      }
      std::cout << " building loop closure constraints for submap: " << id.submapId_ << " resulted in: " << constraints.size()
                << " new constraints \n";
    }
  }
  return retVal;
}

bool SubmapCollection::dumpToFile(const std::string& folderPath, const std::string& filename, const bool& isDenseMap) const {
  bool result = true;
  const size_t n_submaps = submaps_.size();
  for (size_t i = 0; i < n_submaps; ++i) {
    PointCloud copy;
    if (isDenseMap) {
      copy = submaps_.at(i).getDenseMapCopy().toPointCloud();
    } else {
      copy = submaps_.at(i).getMapPointCloudCopy();
    }

    // Assign unique color to all points in submap i
    Eigen::Vector3d color = getColorFromIndex(i, n_submaps);
    copy.colors_.resize(copy.points_.size(), color);

    // File path with .ply extension
    const std::string fullPath = folderPath + "/" + filename + "_" + std::to_string(i) + ".ply";
    result = result && open3d::io::WritePointCloud(fullPath, copy, {/* compressed = false, write_ascii = false, print_progress = false */});
  }

  // Save center spheres
  for (size_t i = 0; i < n_submaps; ++i) {
    Eigen::Vector3d center = submaps_.at(i).getMapToSubmapCenter();
    Eigen::Vector3d color = getColorFromIndex(i, n_submaps);

    // Create a sphere mesh at the submap center
    auto sphere = open3d::geometry::TriangleMesh::CreateSphere(0.3);  // radius in meters, adjust as needed
    sphere->PaintUniformColor(color);
    sphere->Translate(center);

    // Save sphere mesh
    const std::string spherePath = folderPath + "/" + filename + "_center_" + std::to_string(i) + ".ply";
    open3d::io::WriteTriangleMesh(spherePath, *sphere, /*write_ascii=*/false, /*compressed=*/false, /*write_vertex_normals=*/true);
  }

  return result;
}

void SubmapCollection::transform(const OptimizedTransforms& transformIncrements) {
  const size_t nTransforms = transformIncrements.size();
  std::vector<size_t> optimizedIdxs;
  //	std::cout << "Num transforms: " << transformIncrements.size() << std::endl;
  //	std::cout << "Num submaps: " << submaps_.size() << std::endl;
  //	std::cout << "Transforms of updated submaps:: \n";
  for (size_t i = 0; i < nTransforms; ++i) {
    const auto& update = transformIncrements.at(i);
    if (update.submapId_ < submaps_.size()) {
      submaps_.at(update.submapId_).transform(update.dT_);
      optimizedIdxs.push_back(update.submapId_);
      //			std::cout << "Submap " << update.submapId_ << " " << asString(update.dT_) << "\n";
    } else {
      std::cout << "tying to update submap: " << update.submapId_ << " but the there are only: " << submaps_.size()
                << "submaps!!!! This should not happen! \n";
    }
  }
  std::sort(optimizedIdxs.begin(), optimizedIdxs.end());
  std::vector<size_t> allIdxs = getAllSubmapIdxs();

  std::vector<size_t> submapIdxsToUpdate;
  std::set_difference(allIdxs.begin(), allIdxs.end(), optimizedIdxs.begin(), optimizedIdxs.end(),
                      std::inserter(submapIdxsToUpdate, submapIdxsToUpdate.begin()));
  //	std::cout << "\n num maps: " << submaps_.size() << "\n";
  //	std::cout << "num maps missing: " << submapIdxsToUpdate.size() << "\n";
  //	std::cout << " maps that are missing: \n";
  for (auto idx : submapIdxsToUpdate) {
    // look at the node parent
    //  if the parent is not in the list of nodes to update
    //  then you are done, take the transform of the parent node
    //  otherwise, set the current node to be the parent node
    size_t currentNode;
    currentNode = idx;
    //		std::cout << currentNode << " with parent: ";
    while (true && !transformIncrements.empty()) {
      currentNode = submaps_.at(currentNode).getParentId();
      if (std::find(submapIdxsToUpdate.begin(), submapIdxsToUpdate.end(), currentNode) == submapIdxsToUpdate.end()) {
        /* parent is in the pose graph */
        const auto& update = transformIncrements.at(currentNode);
        submaps_.at(idx).transform(update.dT_);
        //				std::cout << update.submapId_ << "\n";
        break;
      }
      if (currentNode == submaps_.at(currentNode).getParentId()) {
        throw std::runtime_error("Stuck in a loop, this should not happen");
      }
    }
  }

  // need to flush the buffered scans
  overlapScansBuffer_.clear();
}

std::vector<size_t> SubmapCollection::getAllSubmapIdxs() const {
  std::vector<size_t> idxs(submaps_.size());
  std::iota(idxs.begin(), idxs.end(), 0);
  return idxs;
}

const MapperParameters& SubmapCollection::getParameters() const {
  return params_;
}

void SubmapCollection::setFolderPath(const std::string& folderPath) {
  savingDataFolderPath_ = folderPath;
  placeRecognition_.setFolderPath(folderPath);
}

bool SubmapCollection::isSwitchingSubmapsConsistant(const PointCloud& scan, size_t cand, const Transform& T) const {
  /* --- read-only fast path: no lock needed -------------------------- */
  const Eigen::Vector3d robot = T.translation();
  const auto& c = consistency_cache_[cand];  // never throws – vector sized

  if (c.valid && (robot - c.last_position).norm() < kConsistencyCheckSpatialThresh)
    return c.last_fitness > params_.submaps_.adjacencyBasedRevisitingMinFitness_;
  /* ------------------------------------------------------------------ */

  /* --- we need to update the slot --> take the lock ----------------- */
  std::lock_guard<std::mutex> lk(consistency_cache_mtx_);

  // re-read after obtaining the lock (double-checked locking)
  auto& cache = consistency_cache_[cand];
  if (cache.valid && (robot - cache.last_position).norm() < kConsistencyCheckSpatialThresh)
    return cache.last_fitness > params_.submaps_.adjacencyBasedRevisitingMinFitness_;

  int inliers = 0;
  const VoxelMap& vox = submaps_[cand].getVoxelMap();
  for (const auto& p_local : scan.points_) {
    Eigen::Vector3d p = T * p_local;
    inliers += static_cast<int>(vox.hasVoxelContainingPoint(p));
  }
  double fitness = static_cast<double>(inliers) / scan.points_.size();

  cache.last_position = robot;
  cache.last_fitness = fitness;
  cache.valid = true;

  return fitness > params_.submaps_.adjacencyBasedRevisitingMinFitness_;
}

}  // namespace o3d_slam
