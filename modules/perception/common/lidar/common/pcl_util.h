/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <string>

#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/lidar/common/lidar_log.h"
#include "modules/perception/common/lidar/common/lidar_point_label.h"

namespace apollo {
namespace perception {
namespace lidar {

typedef pcl::PointXYZRGB CPoint;
typedef pcl::PointCloud<CPoint> CPointCloud;
typedef pcl::PointCloud<CPoint>::Ptr CPointCloudPtr;
typedef pcl::PointCloud<CPoint>::ConstPtr CPointCloudConstPtr;

struct PCLPointXYZIT {
  float x;
  float y;
  float z;
  std::uint8_t intensity;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct PCLPointXYZL {
  float x;
  float y;
  float z;
  uint32_t label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

inline bool LoadPCLPCD(const std::string& file_path,
                       base::PointFCloud* cloud_out) {
  pcl::PointCloud<PCLPointXYZIT> org_cloud;
  if (pcl::io::loadPCDFile(file_path, org_cloud) < 0) {
    AERROR << "Failed to load pcd file " << file_path;
    return false;
  }
  cloud_out->resize(org_cloud.size());
  for (size_t i = 0; i < org_cloud.size(); ++i) {
    auto& pt = org_cloud.points[i];
    auto& new_pt = cloud_out->at(i);
    new_pt.x = pt.x;
    new_pt.y = pt.y;
    new_pt.z = pt.z;
    new_pt.intensity = pt.intensity;
    cloud_out->mutable_points_timestamp()->at(i) = pt.timestamp;
  }
  return true;
}

inline bool WritePcdByLabel(const std::string& file_path,
                            const base::PointFCloud& cloud,
                            const LidarPointLabel& label) {
  pcl::PointCloud<PCLPointXYZIT> pcl_cloud;
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (cloud.points_label().at(i) != static_cast<uint8_t>(label))
      continue;
    PCLPointXYZIT point;
    point.x = cloud[i].x;
    point.y = cloud[i].y;
    point.z = cloud[i].z;
    point.intensity = cloud[i].intensity;
    point.timestamp = cloud.points_timestamp().at(i);
    pcl_cloud.push_back(point);
  }
  try {
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed(file_path, pcl_cloud);
  } catch (const pcl::IOException& ex) {
    AERROR << ex.detailedMessage();
    return false;
  }
  return true;
}

template <typename PointT>
inline void TransformToPCLXYZI(
    const base::AttributePointCloud<PointT>& org_cloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& out_cloud_ptr) {
  for (size_t i = 0; i < org_cloud.size(); ++i) {
    PointT pt = org_cloud.at(i);
    pcl::PointXYZI point;
    point.x = static_cast<float>(pt.x);
    point.y = static_cast<float>(pt.y);
    point.z = static_cast<float>(pt.z);
    point.intensity = static_cast<float>(pt.intensity);
    out_cloud_ptr->push_back(point);
  }
}

inline void TransformFromPCLXYZI(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& org_cloud_ptr,
    const base::PointFCloudPtr& out_cloud_ptr) {
  for (size_t i = 0; i < org_cloud_ptr->size(); ++i) {
    const auto& pt = org_cloud_ptr->at(i);
    base::PointF point;
    point.x = pt.x;
    point.y = pt.y;
    point.z = pt.z;
    point.intensity = pt.intensity;
    out_cloud_ptr->push_back(point);
  }
}

inline void DownSampleCloudByVoxelGrid(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr,
    float lx = 0.01f, float ly = 0.01f, float lz = 0.01f) {
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(cloud_ptr);
  voxel_grid.setLeafSize(lx, ly, lz);
  voxel_grid.filter(*filtered_cloud_ptr);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

POINT_CLOUD_REGISTER_POINT_STRUCT(apollo::perception::lidar::PCLPointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      std::uint8_t, intensity,
                                      intensity)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(apollo::perception::lidar::PCLPointXYZL,
                                  (float, x, x)(float, y, y)(float, z,
                                                             z)(std::uint32_t,
                                                                label, label))
