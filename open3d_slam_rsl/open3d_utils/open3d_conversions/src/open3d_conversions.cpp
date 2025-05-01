// Copyright 2020 Autonomous Robots Lab, University of Nevada, Reno

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "open3d_conversions/open3d_conversions.h"
#include "open3d/core/EigenConverter.h"

namespace open3d_conversions {


  std::shared_ptr<PmStampedPointCloud>
  createSimilarPointmatcherCloud(const std::size_t& size)
  {
      constexpr Eigen::Index kFeat  = 4;
      constexpr Eigen::Index kDescr = 3;
  
      // raw, uninitialised allocation (fast)
      PM::Matrix features(kFeat , static_cast<Eigen::Index>(size));
      PM::Matrix normals  (kDescr, static_cast<Eigen::Index>(size));
  
      // feature labels
      PmDataPoints::Labels featLabels;
      featLabels.reserve(4);
      featLabels.push_back({"x",1});
      featLabels.push_back({"y",1});
      featLabels.push_back({"z",1});
      featLabels.push_back({"pad",1});
  
      PmDataPoints pm(std::move(features), featLabels);
      pm.addDescriptor("normals", std::move(normals));
      pm.getFeatureViewByName("pad").setOnes();   // vectorised
  
      auto cloud   = std::make_shared<PmStampedPointCloud>();
      cloud->dataPoints_ = std::move(pm);
      cloud->header_ = std_msgs::Header();
      return cloud;
  }
  

  void open3dToPointmatcher(const open3d::geometry::PointCloud& src,
    PmStampedPointCloud& dst,
    bool copyNormals)
{
const std::size_t N = src.points_.size();
if (dst.isEmpty() || N != static_cast<std::size_t>(dst.dataPoints_.features.cols()))
return;

// --- copy XYZ (Eigen does SIMD) -----------------------------------------
auto xyzDst = dst.dataPoints_.features.topRows(3);
Eigen::Map<const Eigen::Matrix<double,3,Eigen::Dynamic,Eigen::ColMajor>>
xyzSrc(reinterpret_cast<const double*>(src.points_.data()),3,
static_cast<Eigen::Index>(N));
xyzDst = xyzSrc.cast<float>();

// --- copy normals if requested ------------------------------------------
if (copyNormals && src.HasNormals()) {
PmDataPoints::View nDst = dst.dataPoints_.getDescriptorViewByName("normals");
Eigen::Map<const Eigen::Matrix<double,3,Eigen::Dynamic,Eigen::ColMajor>>
nSrc(reinterpret_cast<const double*>(src.normals_.data()),3,
static_cast<Eigen::Index>(N));
nDst = nSrc.cast<float>();
}
}


void pointmatcherToOpen3d(const PmStampedPointCloud& pointMatcherCloud,
  open3d::geometry::PointCloud& pointcloud)
{
if (!pointMatcherCloud.descriptorExists("normals") || pointMatcherCloud.isEmpty())
return;

const auto& F = pointMatcherCloud.dataPoints_.features;
const auto  Nview = pointMatcherCloud.dataPoints_.getDescriptorViewByName("normals");

const std::size_t N = static_cast<std::size_t>(F.cols());
pointcloud.points_.resize(N);
pointcloud.normals_.resize(N);

#pragma omp parallel for schedule(static)
for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(N); ++i) {
Eigen::Vector3d p;
p << F(0,i), F(1,i), F(2,i);
pointcloud.points_[i] = p;

Eigen::Vector3d n;
n << static_cast<double>(Nview(0,i)),
static_cast<double>(Nview(1,i)),
static_cast<double>(Nview(2,i));
pointcloud.normals_[i] = n;
}
}



void open3dToRos(const open3d::geometry::PointCloud& pointcloud, sensor_msgs::PointCloud2& ros_pc2, std::string frame_id) {
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  bool has_colors = pointcloud.HasColors();

  if (has_colors) {
      modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
      modifier.setPointCloud2FieldsByString(1, "xyz");
  }

  const size_t num_points = pointcloud.points_.size();
  modifier.resize(num_points);

  ros_pc2.header.frame_id = frame_id;
  ros_pc2.height = 1;
  ros_pc2.width = num_points;
  ros_pc2.is_dense = true;

  const uint8_t* data_ptr = ros_pc2.data.data();
  const size_t point_step = ros_pc2.point_step;

  int x_offset = -1, y_offset = -1, z_offset = -1;
  int r_offset = -1, g_offset = -1, b_offset = -1;

  for (const auto& field : ros_pc2.fields) {
      if (field.name == "x") x_offset = field.offset;
      else if (field.name == "y") y_offset = field.offset;
      else if (field.name == "z") z_offset = field.offset;
      else if (field.name == "r") r_offset = field.offset;
      else if (field.name == "g") g_offset = field.offset;
      else if (field.name == "b") b_offset = field.offset;
  }

  #pragma omp parallel for simd
  for (int i = 0; i < static_cast<int>(num_points); ++i) {
      uint8_t* ptr = ros_pc2.data.data() + i * point_step;

      const Eigen::Vector3d& p = pointcloud.points_[i];
      *reinterpret_cast<float*>(ptr + x_offset) = static_cast<float>(p.x());
      *reinterpret_cast<float*>(ptr + y_offset) = static_cast<float>(p.y());
      *reinterpret_cast<float*>(ptr + z_offset) = static_cast<float>(p.z());

      if (has_colors) {
          const Eigen::Vector3d& c = pointcloud.colors_[i];
          *(ptr + r_offset) = static_cast<uint8_t>(std::min(255.0, std::max(0.0, 255.0 * c.x())));
          *(ptr + g_offset) = static_cast<uint8_t>(std::min(255.0, std::max(0.0, 255.0 * c.y())));
          *(ptr + b_offset) = static_cast<uint8_t>(std::min(255.0, std::max(0.0, 255.0 * c.z())));
      }
  }
}


bool rosToOpen3d(const sensor_msgs::PointCloud2ConstPtr& ros_pc2, open3d::geometry::PointCloud& o3d_pc, bool skip_colors,  bool copy_normals) {
  return rosToOpen3d(*ros_pc2, o3d_pc, skip_colors, copy_normals);
}

bool rosToOpen3d(const sensor_msgs::PointCloud2& cloud, open3d::geometry::PointCloud& o3d_pc, bool skip_colors, bool copy_normals) {
  const uint32_t num_points = cloud.height * cloud.width;
  if (cloud.fields.size() < 3 || num_points == 0) {
      std::cout << "ros_pc2->fields.size(): " << cloud.fields.size() << " Exiting." << std::endl;
      return false;
  }

  int x_offset = -1, y_offset = -1, z_offset = -1;
  int normal_x_offset = -1, normal_y_offset = -1, normal_z_offset = -1;
  int r_offset = -1, g_offset = -1, b_offset = -1;
  bool has_rgb = false;
  bool has_normals = false;

  for (const auto& field : cloud.fields) {
      if (field.name == "x") x_offset = field.offset;
      else if (field.name == "y") y_offset = field.offset;
      else if (field.name == "z") z_offset = field.offset;
      else if (field.name == "normal_x") normal_x_offset = field.offset;
      else if (field.name == "normal_y") normal_y_offset = field.offset;
      else if (field.name == "normal_z") normal_z_offset = field.offset;
      else if (field.name == "r") r_offset = field.offset;
      else if (field.name == "g") g_offset = field.offset;
      else if (field.name == "b") b_offset = field.offset;
  }

  has_normals = (normal_x_offset >= 0 && normal_y_offset >= 0 && normal_z_offset >= 0);
  has_rgb = (r_offset >= 0 && g_offset >= 0 && b_offset >= 0);

  const uint8_t* base_ptr = cloud.data.data();
  const size_t point_step = cloud.point_step;

  o3d_pc.points_.resize(num_points);

  bool can_batch_load_xyz = (x_offset == 0) && (y_offset == 4) && (z_offset == 8) && (point_step >= 12);

  if (can_batch_load_xyz) {
      // Fast path: batch load x,y,z in one shot
      #pragma omp parallel for simd
      for (int i = 0; i < static_cast<int>(num_points); ++i) {
          Eigen::Vector3f temp;
          std::memcpy(&temp, base_ptr + i * point_step, 12);
          o3d_pc.points_[i] = temp.cast<double>();
      }
  } else {
      // Fallback: standard safe load
      #pragma omp parallel for simd
      for (int i = 0; i < static_cast<int>(num_points); ++i) {
          const uint8_t* ptr = base_ptr + i * point_step;
          float x = *reinterpret_cast<const float*>(ptr + x_offset);
          float y = *reinterpret_cast<const float*>(ptr + y_offset);
          float z = *reinterpret_cast<const float*>(ptr + z_offset);
          o3d_pc.points_[i] = Eigen::Vector3d(x, y, z);
      }
  }

  if (copy_normals && has_normals) {
      o3d_pc.normals_.resize(num_points);
      #pragma omp parallel for simd
      for (int i = 0; i < static_cast<int>(num_points); ++i) {
          const uint8_t* ptr = base_ptr + i * point_step;
          float nx = *reinterpret_cast<const float*>(ptr + normal_x_offset);
          float ny = *reinterpret_cast<const float*>(ptr + normal_y_offset);
          float nz = *reinterpret_cast<const float*>(ptr + normal_z_offset);
          o3d_pc.normals_[i] = Eigen::Vector3d(nx, ny, nz);
      }
  }

  if (skip_colors) {
      return true;
  }

  if (has_rgb) {
      o3d_pc.colors_.resize(num_points);
      #pragma omp parallel for simd
      for (int i = 0; i < static_cast<int>(num_points); ++i) {
          const uint8_t* ptr = base_ptr + i * point_step;
          uint8_t r = *(ptr + r_offset);
          uint8_t g = *(ptr + g_offset);
          uint8_t b = *(ptr + b_offset);
          o3d_pc.colors_[i] = Eigen::Vector3d(
              static_cast<double>(r) / 255.0,
              static_cast<double>(g) / 255.0,
              static_cast<double>(b) / 255.0
          );
      }
  }

  return true;
}

void open3dToRos(const open3d::t::geometry::PointCloud& pointcloud, sensor_msgs::PointCloud2& ros_pc2, std::string frame_id,
                 int t_num_fields, ...) {
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  ros_pc2.fields.reserve(t_num_fields);
  va_list vl;
  va_start(vl, t_num_fields);
  int offset = 0;
  std::vector<std::string> field_names;
  std::vector<std::string> data_types;
  for (int i = 0; i < t_num_fields - 1; ++i) {
    std::string field_name = std::string(va_arg(vl, char*));
    field_names.push_back(field_name);
    i++;
    std::string data_type = std::string(va_arg(vl, char*));
    data_types.push_back(data_type);
    if (field_name == "xyz") {
      modifier.setPointCloud2FieldsByString(1, "xyz");
    } else {
      if ((field_name == "rgb") || (field_name == "rgba")) {
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
      } else {
        if (data_type == "float") {
          offset = addPointField(ros_pc2, field_name + "_x", 1, sensor_msgs::PointField::FLOAT32, offset);
          offset = addPointField(ros_pc2, field_name + "_y", 1, sensor_msgs::PointField::FLOAT32, offset);
          offset = addPointField(ros_pc2, field_name + "_z", 1, sensor_msgs::PointField::FLOAT32, offset);
          offset += sizeOfPointField(sensor_msgs::PointField::FLOAT32);
        } else if (data_type == "int") {
          offset = addPointField(ros_pc2, field_name + "_x", 1, sensor_msgs::PointField::INT8, offset);
          offset = addPointField(ros_pc2, field_name + "_y", 1, sensor_msgs::PointField::INT8, offset);
          offset = addPointField(ros_pc2, field_name + "_z", 1, sensor_msgs::PointField::INT8, offset);
          offset += sizeOfPointField(sensor_msgs::PointField::INT8);
        } else {
          throw std::runtime_error("datatype" + data_type + " does not exist");
        }
      }
    }
    va_end(vl);
  }
  const open3d::core::Tensor& o3d_TensorList_points = pointcloud.GetPointPositions();
  modifier.resize(pointcloud.GetPointPositions().GetShape()[0]);
  ros_pc2.data.reserve(pointcloud.GetPointPositions().GetShape()[0]);
  ros_pc2.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
  int count = 0;
  for (auto field_name = field_names.begin(); field_name != field_names.end(); ++field_name) {
    std::string data_type = data_types[count];
    if (*field_name == "xyz") {
      for (size_t i = 0; i < o3d_TensorList_points.GetShape()[0]; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
        open3d::core::Tensor point = o3d_TensorList_points[i];
        *ros_pc2_x = point[0].Item<float>();
        *ros_pc2_y = point[1].Item<float>();
        *ros_pc2_z = point[2].Item<float>();
      }
    } else if (*field_name == "rgb") {
      const open3d::core::Tensor& o3d_TensorList_colors = pointcloud.GetPointAttr("colors");
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
      for (size_t i = 0; i < o3d_TensorList_points.GetShape()[0]; i++, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b) {
        open3d::core::Tensor color = o3d_TensorList_colors[i];
        *ros_pc2_r = (int)(255 * color[0].Item<float>());
        *ros_pc2_g = (int)(255 * color[1].Item<float>());
        *ros_pc2_b = (int)(255 * color[2].Item<float>());
      }
    } else {
      const open3d::core::Tensor& o3d_TensorList_fields = pointcloud.GetPointAttr(*field_name);
      if (data_type == "int") {
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_fx(ros_pc2, *field_name + "_x");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_fy(ros_pc2, *field_name + "_y");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_fz(ros_pc2, *field_name + "_z");
        for (size_t i = 0; i < o3d_TensorList_points.GetShape()[0];
             i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_fx, ++ros_pc2_fy, ++ros_pc2_fz) {
          open3d::core::Tensor field_tensor = o3d_TensorList_points[i];
          *ros_pc2_fx = field_tensor[0].Item<int>();
          *ros_pc2_fy = field_tensor[1].Item<int>();
          *ros_pc2_fz = field_tensor[2].Item<int>();
        }
      } else if (data_type == "float") {
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_fx(ros_pc2, *field_name + "_x");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_fy(ros_pc2, *field_name + "_y");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_fz(ros_pc2, *field_name + "_z");
        for (size_t i = 0; i < o3d_TensorList_points.GetShape()[0];
             i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_fx, ++ros_pc2_fy, ++ros_pc2_fz) {
          open3d::core::Tensor field_tensor = o3d_TensorList_points[i];
          *ros_pc2_fx = field_tensor[0].Item<float>();
          *ros_pc2_fy = field_tensor[1].Item<float>();
          *ros_pc2_fz = field_tensor[2].Item<float>();
        }
      }
    }
    count++;
  }
}

void rosToOpen3d(const sensor_msgs::PointCloud2ConstPtr& ros_pc2, open3d::t::geometry::PointCloud& o3d_tpc, bool skip_colors) {
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");
  open3d::core::Dtype dtype_f = open3d::core::Dtype::Float32;
  open3d::core::Dtype dtype_lf = open3d::core::Dtype::Float64;
  open3d::core::Device device_type(open3d::core::Device::DeviceType::CPU, 0);
  std::vector<Eigen::Vector3d> o3d_TensorList_points;

  for (int num_fields = 0; num_fields < ros_pc2->fields.size(); num_fields++) {
    if (ros_pc2->fields[num_fields].name == "x") {
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
        o3d_TensorList_points.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
      }
      open3d::core::Tensor o3d_tpc_points =
          open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_TensorList_points, dtype_f, device_type);
      o3d_tpc.SetPointPositions(o3d_tpc_points);
      num_fields++;
      num_fields++;
    } else if (ros_pc2->fields[num_fields].name == "rgb" && !skip_colors) {
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(*ros_pc2, "r");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(*ros_pc2, "g");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(*ros_pc2, "b");
      std::vector<Eigen::Vector3d> o3d_TensorList_colors;
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b) {
        o3d_TensorList_colors.push_back(
            Eigen::Vector3d(((int)(*ros_pc2_r)) / 255.0, ((int)(*ros_pc2_g)) / 255.0, ((int)(*ros_pc2_b)) / 255.0));
      }
      open3d::core::Tensor o3d_tpc_colors =
          open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_TensorList_colors, dtype_f, device_type);
      o3d_tpc.SetPointColors(o3d_tpc_colors);
    } else {
      if (ros_pc2->fields[num_fields].datatype == sensor_msgs::PointField::UINT8 ||
          ros_pc2->fields[num_fields].datatype == sensor_msgs::PointField::INT8) {
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_fx(*ros_pc2, ros_pc2->fields[num_fields].name);
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_fy(*ros_pc2, ros_pc2->fields[num_fields].name);
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_fz(*ros_pc2, ros_pc2->fields[num_fields].name);
        std::vector<Eigen::Vector3d> o3d_TensorList_fields;

        for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_fx, ++ros_pc2_fy, ++ros_pc2_fz) {
          o3d_TensorList_fields.push_back(Eigen::Vector3d(*ros_pc2_fx, *ros_pc2_fy, *ros_pc2_fz));
        }
        open3d::core::Tensor o3d_tpc_fields =
            open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_TensorList_fields, dtype_f, device_type);
        o3d_tpc.SetPointAttr(ros_pc2->fields[num_fields].name, o3d_tpc_fields);
      } else if (ros_pc2->fields[num_fields].datatype == sensor_msgs::PointField::FLOAT32) {
        sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_fx(*ros_pc2, ros_pc2->fields[num_fields].name);
        sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_fy(*ros_pc2, ros_pc2->fields[num_fields].name);
        sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_fz(*ros_pc2, ros_pc2->fields[num_fields].name);
        std::vector<Eigen::Vector3d> o3d_TensorList_fields;

        for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_fx, ++ros_pc2_fy, ++ros_pc2_fz) {
          o3d_TensorList_fields.push_back(Eigen::Vector3d(*ros_pc2_fx, *ros_pc2_fy, *ros_pc2_fz));
        }
        open3d::core::Tensor o3d_tpc_fields =
            open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_TensorList_fields, dtype_f, device_type);
        o3d_tpc.SetPointAttr(ros_pc2->fields[num_fields].name, o3d_tpc_fields);
      } else {
        sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_fx(*ros_pc2, ros_pc2->fields[num_fields].name);
        sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_fy(*ros_pc2, ros_pc2->fields[num_fields].name);
        sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_fz(*ros_pc2, ros_pc2->fields[num_fields].name);
        std::vector<Eigen::Vector3d> o3d_TensorList_fields;

        for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_fx, ++ros_pc2_fy, ++ros_pc2_fz) {
          o3d_TensorList_fields.push_back(Eigen::Vector3d(*ros_pc2_fx, *ros_pc2_fy, *ros_pc2_fz));
        }
        open3d::core::Tensor o3d_tpc_fields =
            open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_TensorList_fields, dtype_f, device_type);
        o3d_tpc.SetPointAttr(ros_pc2->fields[num_fields].name, o3d_tpc_fields);
      }
    }
  }
}

void open3dToRos(const open3d::geometry::MeshBase& mesh, const std::string& frameId, open3d_slam_msgs::PolygonMesh& msg) {
  enum XYZ { x, y, z };
  // Constructing the vertices pointcloud
  using namespace open3d::geometry;
  PointCloud pointCloud;
  const int nVertices = mesh.vertices_.size();
  pointCloud.points_ = mesh.vertices_;
  open3dToRos(pointCloud, msg.cloud, frameId);

  if (mesh.GetGeometryType() != Geometry::GeometryType::TriangleMesh) {
    throw std::runtime_error("Only triangle mesh supported for now!");
  }

  const TriangleMesh& triangleMesh = static_cast<const TriangleMesh&>(mesh);

  // add triangles
  const int nTriangles = triangleMesh.triangles_.size();
  msg.polygons.reserve(nTriangles);
  for (int i = 0; i < nTriangles; ++i) {
    open3d_slam_msgs::Vertices triangle;
    triangle.vertices.resize(3);
    triangle.vertices[x] = triangleMesh.triangles_.at(i).x();
    triangle.vertices[y] = triangleMesh.triangles_.at(i).y();
    triangle.vertices[z] = triangleMesh.triangles_.at(i).z();
    msg.polygons.push_back(triangle);
  }
}

void rosToOpen3d(const open3d_slam_msgs::PolygonMesh::ConstPtr& msg, open3d::geometry::TriangleMesh& mesh) {
  rosToOpen3d(*msg, mesh);
}

void rosToOpen3d(const open3d_slam_msgs::PolygonMesh& msg, open3d::geometry::TriangleMesh& mesh) {
  using namespace open3d::geometry;
  enum XYZ { x, y, z };
  PointCloud pointCloud;

  rosToOpen3d(msg.cloud, pointCloud);
  mesh.vertices_ = pointCloud.points_;

  // add triangles
  const int nTriangles = msg.polygons.size();
  mesh.triangles_.reserve(nTriangles);
  for (int i = 0; i < nTriangles; ++i) {
    Eigen::Vector3i triangle(msg.polygons[i].vertices[x], msg.polygons[i].vertices[y], msg.polygons[i].vertices[z]);
    mesh.triangles_.push_back(triangle);
  }
}

}  // namespace open3d_conversions

inline int addPointField(sensor_msgs::PointCloud2& cloud_msg, const std::string& name, int count, int datatype, int offset)

{
  sensor_msgs::PointField point_field;
  point_field.name = name;
  point_field.count = count;
  point_field.datatype = datatype;
  point_field.offset = offset;
  cloud_msg.fields.push_back(point_field);

  // Update the offset
  return offset + point_field.count * sizeOfPointField(datatype);
}

inline int sizeOfPointField(int datatype) {
  if ((datatype == sensor_msgs::PointField::INT8) || (datatype == sensor_msgs::PointField::UINT8)) {
    return 1;
  } else if ((datatype == sensor_msgs::PointField::INT16) ||  // NOLINT
             (datatype == sensor_msgs::PointField::UINT16)) {
    return 2;
  } else if ((datatype == sensor_msgs::PointField::INT32) ||  // NOLINT
             (datatype == sensor_msgs::PointField::UINT32) || (datatype == sensor_msgs::PointField::FLOAT32)) {
    return 4;
  } else if (datatype == sensor_msgs::PointField::FLOAT64) {
    return 8;
  } else {
    std::stringstream err;
    err << "PointField of type " << datatype << " does not exist";
    throw std::runtime_error(err.str());
  }
  return -1;
}
