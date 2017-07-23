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

#include <Eigen/Core>
#include <algorithm>
#include <float.h>

#include "modules/perception/lib/pcl_util/pcl_types.h"

namespace apollo {
namespace perception {

void TransAffineToMatrix4(const Eigen::Vector3d& translation,
                          const Eigen::Vector4d& rotation,
                          Eigen::Matrix4d* trans_matrix);

template <typename PointT>
void TransformPointCloud(const Eigen::Matrix4d& trans_mat,
                         typename pcl::PointCloud<PointT>::Ptr cloud_in_out) {
  for (int i = 0; i < cloud_in_out->size(); ++i) {
    PointT& p = cloud_in_out->at(i);
    Eigen::Vector4d v(p.x, p.y, p.z, 1);
    v = trans_mat * v;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
  }
}

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

void TransformCloud(pcl_util::PointCloudPtr cloud,
    std::vector<int> indices,
    pcl_util::PointDCloud& trans_cloud);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_COMMON_GEOMETRY_UTIL_H_
