/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_OBSTACLE_COMMON_GEOMETRY_UTIL_H_
#define MODULES_PERCEPTION_OBSTACLE_COMMON_GEOMETRY_UTIL_H_

#include <cfloat>

#include <algorithm>
#include <vector>

#include "Eigen/Core"

#include "modules/common/log.h"
#include "modules/perception/common/pcl_types.h"

namespace apollo {
namespace perception {

template <typename PointT>
void TransformPointCloud(const Eigen::Matrix4d& trans_mat,
                         pcl::PointCloud<PointT>* cloud_in_out) {
  for (std::size_t i = 0; i < cloud_in_out->size(); ++i) {
    PointT& p = cloud_in_out->at(i);
    Eigen::Vector4d v(p.x, p.y, p.z, 1);
    v = trans_mat * v;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
  }
}

template <typename PointT>
void TransformPointCloud(const Eigen::Matrix4d& trans_mat,
                         typename pcl::PointCloud<PointT>::Ptr cloud_in_out) {
  CHECK_NOTNULL(cloud_in_out.get());
  return TransformPointCloud(trans_mat, cloud_in_out.get());
}

template <typename PointType>
void TransformPoint(const PointType& point_in, const Eigen::Matrix4d& trans_mat,
                    PointType* point_out) {
  Eigen::Vector4d v(point_in.x, point_in.y, point_in.z, 1);
  v = trans_mat * v;
  *point_out = point_in;
  point_out->x = v.x();
  point_out->y = v.y();
  point_out->z = v.z();
}

template <typename PointType>
void TransformPointCloud(const pcl::PointCloud<PointType>& cloud_in,
                         const Eigen::Matrix4d& trans_mat,
                         pcl::PointCloud<PointType>* cloud_out) {
  if (cloud_out->points.size() < cloud_in.points.size()) {
    cloud_out->points.resize(cloud_in.points.size());
  }
  for (std::size_t i = 0; i < cloud_in.size(); ++i) {
    const PointType& p = cloud_in.at(i);
    Eigen::Vector4d v(p.x, p.y, p.z, 1);
    v = trans_mat * v;
    PointType& pd = cloud_out->points[i];
    pd.x = v.x();
    pd.y = v.y();
    pd.z = v.z();
  }
}

void TransformPointCloud(pcl_util::PointCloudPtr cloud,
                         const std::vector<int>& indices,
                         pcl_util::PointDCloud* trans_cloud);

void TransformPointCloud(pcl_util::PointCloudPtr cloud,
                         const Eigen::Matrix4d& pose_velodyne,
                         typename pcl_util::PointDCloudPtr trans_cloud);
/*
 * Other point cloud related methods
 * */
template <typename PointT>
void GetCloudMinMax3D(typename pcl::PointCloud<PointT>::Ptr cloud,
                      Eigen::Vector4f* min_point, Eigen::Vector4f* max_point) {
  Eigen::Vector4f& min_pt = *min_point;
  Eigen::Vector4f& max_pt = *max_point;
  min_pt[0] = min_pt[1] = min_pt[2] = FLT_MAX;
  max_pt[0] = max_pt[1] = max_pt[2] = -FLT_MAX;
  if (cloud->is_dense) {
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      min_pt[0] = std::min(min_pt[0], cloud->points[i].x);
      max_pt[0] = std::max(max_pt[0], cloud->points[i].x);
      min_pt[1] = std::min(min_pt[1], cloud->points[i].y);
      max_pt[1] = std::max(max_pt[1], cloud->points[i].y);
      min_pt[2] = std::min(min_pt[2], cloud->points[i].z);
      max_pt[2] = std::max(max_pt[2], cloud->points[i].z);
    }
  } else {
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      if (!pcl_isfinite(cloud->points[i].x) ||
          !pcl_isfinite(cloud->points[i].y) ||
          !pcl_isfinite(cloud->points[i].z)) {
        continue;
      }
      min_pt[0] = std::min(min_pt[0], cloud->points[i].x);
      max_pt[0] = std::max(max_pt[0], cloud->points[i].x);
      min_pt[1] = std::min(min_pt[1], cloud->points[i].y);
      max_pt[1] = std::max(max_pt[1], cloud->points[i].y);
      min_pt[2] = std::min(min_pt[2], cloud->points[i].z);
      max_pt[2] = std::max(max_pt[2], cloud->points[i].z);
    }
  }
}

template <typename PointT>
void ComputeBboxSizeCenter(typename pcl::PointCloud<PointT>::Ptr cloud,
                           const Eigen::Vector3d& direction,
                           Eigen::Vector3d* size, Eigen::Vector3d* center) {
  Eigen::Vector3d dir(direction[0], direction[1], 0);
  dir.normalize();
  Eigen::Vector3d ortho_dir(-dir[1], dir[0], 0.0);

  Eigen::Vector3d z_dir(dir.cross(ortho_dir));
  Eigen::Vector3d min_pt(DBL_MAX, DBL_MAX, DBL_MAX);
  Eigen::Vector3d max_pt(-DBL_MAX, -DBL_MAX, -DBL_MAX);

  Eigen::Vector3d loc_pt;
  for (std::size_t i = 0; i < cloud->size(); i++) {
    Eigen::Vector3d pt = Eigen::Vector3d(cloud->points[i].x, cloud->points[i].y,
                                         cloud->points[i].z);
    loc_pt[0] = pt.dot(dir);
    loc_pt[1] = pt.dot(ortho_dir);
    loc_pt[2] = pt.dot(z_dir);
    for (int j = 0; j < 3; j++) {
      min_pt[j] = std::min(min_pt[j], loc_pt[j]);
      max_pt[j] = std::max(max_pt[j], loc_pt[j]);
    }
  }
  *size = max_pt - min_pt;
  *center = dir * ((max_pt[0] + min_pt[0]) * 0.5) +
            ortho_dir * ((max_pt[1] + min_pt[1]) * 0.5) + z_dir * min_pt[2];
}

template <typename PointT>
Eigen::Vector3d GetCloudBarycenter(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  int point_num = cloud->points.size();
  Eigen::Vector3d barycenter(0, 0, 0);

  for (int i = 0; i < point_num; i++) {
    const PointT& pt = cloud->points[i];
    barycenter[0] += pt.x;
    barycenter[1] += pt.y;
    barycenter[2] += pt.z;
  }

  if (point_num > 0) {
    barycenter[0] /= point_num;
    barycenter[1] /= point_num;
    barycenter[2] /= point_num;
  }
  return barycenter;
}

void TransAffineToMatrix4(const Eigen::Vector3d& translation,
                          const Eigen::Vector4d& rotation,
                          Eigen::Matrix4d* trans_matrix);

void ComputeMostConsistentBboxDirection(const Eigen::Vector3f& previous_dir,
                                        Eigen::Vector3f* current_dir);

double VectorCosTheta2dXy(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);

double VectorTheta2dXy(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_COMMON_GEOMETRY_UTIL_H_
