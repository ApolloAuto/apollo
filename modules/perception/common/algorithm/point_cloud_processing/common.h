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

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include "Eigen/Core"

#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/base/radar_point_cloud.h"

namespace apollo {
namespace perception {
namespace algorithm {

// @brief transform a point
// old name: transform_point_cloud
template <typename PointT>
void TransformPoint(const PointT &point_in, const Eigen::Affine3d &pose,
                    PointT *point_out) {
  Eigen::Vector3d point3d(point_in.x, point_in.y, point_in.z);
  point3d = pose * point3d;
  point_out->x = static_cast<typename PointT::Type>(point3d.x());
  point_out->y = static_cast<typename PointT::Type>(point3d.y());
  point_out->z = static_cast<typename PointT::Type>(point3d.z());
}

// @brief transform a point cloud
// old name: transform_point_cloud
template <typename PointT>
void TransformPointCloud(const base::PointCloud<PointT> &cloud_in,
                         const Eigen::Affine3d &pose,
                         base::PointCloud<PointT> *cloud_out) {
  if (cloud_out->size() < cloud_in.size()) {
    cloud_out->resize(cloud_in.size());
  }
  for (size_t i = 0; i < cloud_in.size(); ++i) {
    TransformPoint<PointT>(cloud_in.at(i), pose, &(cloud_out->at(i)));
  }
}

// @brief transform a point cloud
// old name:transform_point_cloud
template <typename PointT>
void TransformPointCloud(const Eigen::Affine3d &pose,
                         base::PointCloud<PointT> *cloud_in_out) {
  TransformPointCloud<PointT>(*cloud_in_out, pose, cloud_in_out);
}

// @brief extract the indexed points from a point cloud
// old name: transform_cloud
template <typename PointCloudT>
void ExtractIndicedCloud(const std::shared_ptr<const PointCloudT> cloud,
                         const std::vector<int> &indices,
                         std::shared_ptr<PointCloudT> trans_cloud) {
  if (trans_cloud->size() != indices.size()) {
    trans_cloud->resize(indices.size());
  }
  for (size_t i = 0; i < indices.size(); ++i) {
    const auto &p = cloud->at(indices[i]);
    auto &tp = trans_cloud->at(i);
    tp.x = p.x;
    tp.y = p.y;
    tp.z = p.z;
    tp.intensity = p.intensity;
  }
}

// @brief get the maximum and minimum in each axis of a point cloud
// old name: du_get_min_max_3d
template <typename PointT>
void GetMinMaxIn3DWithRange(const base::AttributePointCloud<PointT> &cloud,
                            const size_t range,
                            Eigen::Matrix<typename PointT::Type, 4, 1> *min_p,
                            Eigen::Matrix<typename PointT::Type, 4, 1> *max_p) {
  using T = typename PointT::Type;
  (*min_p)[0] = (*min_p)[1] = (*min_p)[2] = std::numeric_limits<T>::max();
  (*max_p)[0] = (*max_p)[1] = (*max_p)[2] = -std::numeric_limits<T>::max();
  (*min_p)[3] = 0.0;
  (*max_p)[3] = 0.0;
  for (size_t i = 0; i < range; ++i) {
    const auto &pt = cloud.at(i);
    if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
      continue;
    }
    (*min_p)[0] = std::min((*min_p)[0], static_cast<T>(pt.x));
    (*max_p)[0] = std::max((*max_p)[0], static_cast<T>(pt.x));
    (*min_p)[1] = std::min((*min_p)[1], static_cast<T>(pt.y));
    (*max_p)[1] = std::max((*max_p)[1], static_cast<T>(pt.y));
    (*min_p)[2] = std::min((*min_p)[2], static_cast<T>(pt.z));
    (*max_p)[2] = std::max((*max_p)[2], static_cast<T>(pt.z));
  }
}

// @brief get the maximum and minimum in each axis of an indexed point cloud
// old name: du_get_min_max_3d
template <typename PointT>
void GetMinMaxIn3D(const base::AttributePointCloud<PointT> &cloud,
                   const base::PointIndices &indices,
                   Eigen::Matrix<typename PointT::Type, 4, 1> *min_p,
                   Eigen::Matrix<typename PointT::Type, 4, 1> *max_p) {
  GetMinMaxIn3DWithRange<PointT>(cloud, indices.indices.size(), min_p, max_p);
}

// @brief get the maximum and minimum in each axis of a point cloud
// old name: du_get_min_max_3d
template <typename PointT>
void GetMinMaxIn3D(const base::AttributePointCloud<PointT> &cloud,
                   Eigen::Matrix<typename PointT::Type, 4, 1> *min_p,
                   Eigen::Matrix<typename PointT::Type, 4, 1> *max_p) {
  GetMinMaxIn3DWithRange<PointT>(cloud, cloud.size(), min_p, max_p);
}

// @brief calculate the centroid of a point cloud
// old name: get_barycenter
template <typename T>
Eigen::Matrix<T, 3, 1> CalculateCentroid(
    const base::AttributePointCloud<base::Point<T>> &cloud) {
  size_t point_num = cloud.size();
  Eigen::Matrix<T, 3, 1> centroid(0.0, 0.0, 0.0);
  for (const auto &pt : cloud.points()) {
    centroid[0] += pt.x;
    centroid[1] += pt.y;
    centroid[2] += pt.z;
  }
  if (point_num > 0) {
    centroid[0] /= static_cast<T>(point_num);
    centroid[1] /= static_cast<T>(point_num);
    centroid[2] /= static_cast<T>(point_num);
  }
  return centroid;
}

template <typename T>
Eigen::Matrix<T, 3, 1> CalculateRadarCentroid(
    const base::AttributeRadarPointCloud<base::RadarPoint<T>> &cloud) {
  size_t point_num = cloud.size();
  Eigen::Matrix<T, 3, 1> centroid(0.0, 0.0, 0.0);
  for (const auto &pt : cloud.points()) {
    centroid[0] += pt.x;
    centroid[1] += pt.y;
    centroid[2] += pt.z;
  }
  if (point_num > 0) {
    centroid[0] /= static_cast<T>(point_num);
    centroid[1] /= static_cast<T>(point_num);
    centroid[2] /= static_cast<T>(point_num);
  }
  return centroid;
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
