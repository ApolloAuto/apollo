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
#ifndef PERCEPTION_COMMON_POINT_CLOUD_PROCESSING_COMMON_H_
#define PERCEPTION_COMMON_POINT_CLOUD_PROCESSING_COMMON_H_
#include <Eigen/Core>
#include <algorithm>
#include <limits>
#include <vector>
#include "modules/perception/base/point_cloud_types.h"
namespace apollo {
namespace perception {
namespace common {

// @brief transform a point
// old name: transform_point_cloud
template <typename PointT>
void TransformPoint(const PointT &point_in, const Eigen::Affine3d &pose,
                    PointT *point_out) {
  Eigen::Vector3d point3d(point_in.x, point_in.y, point_in.z);
  point3d = pose * point3d;
  point_out->x = point3d.x();
  point_out->y = point3d.y();
  point_out->z = point3d.z();
}

// @brief transform a point cloud
// old name: transform_point_cloud
template <typename PointCloudT>
void TransformPointCloud(const PointCloudT &cloud_in,
                         const Eigen::Affine3d &pose, PointCloudT *cloud_out) {
  if (cloud_out->size() < cloud_in.size()) {
    cloud_out->resize(cloud_in.size());
  }
  for (size_t i = 0; i < cloud_in.size(); ++i) {
    const auto &p = cloud_in[i];
    Eigen::Vector3d point3d(p.x, p.y, p.z);
    point3d = pose * point3d;
    auto &pd = (*cloud_out)[i];
    pd.x = point3d.x();
    pd.y = point3d.y();
    pd.z = point3d.z();
  }
}

// @brief transform a point cloud
// old name: transform_point_cloud
template <typename PointCloudT>
void TransformPointCloud(const std::shared_ptr<const PointCloudT> cloud_in,
                         const Eigen::Affine3d &pose,
                         std::shared_ptr<PointCloudT> cloud_out) {
  if (cloud_out->size() < cloud_in->size()) {
    cloud_out->resize(cloud_in->size());
  }
  for (size_t i = 0; i < cloud_in->size(); ++i) {
    const auto &p = cloud_in->at(i);
    Eigen::Vector3d point3d(p.x, p.y, p.z);
    point3d = pose * point3d;
    auto &pd = cloud_out->at(i);
    pd.x = point3d.x();
    pd.y = point3d.y();
    pd.z = point3d.z();
  }
}

// @brief transform a point cloud
// old name:transform_point_cloud
template <typename PointCloudT>
void TransformPointCloud(const Eigen::Affine3d &pose,
                         PointCloudT *cloud_in_out) {
  for (size_t i = 0; i < cloud_in_out->size(); ++i) {
    auto &p = cloud_in_out->at(i);
    Eigen::Vector3d point3d(p.x, p.y, p.z);
    point3d = pose * point3d;
    p.x = point3d.x();
    p.y = point3d.y();
    p.z = point3d.z();
  }
}

// @brief transform a point cloud
// old name:transform_point_cloud
template <typename PointCloudT>
void TransformPointCloud(const Eigen::Affine3d &pose,
                         std::shared_ptr<PointCloudT> cloud_in_out) {
  for (size_t i = 0; i < cloud_in_out->size(); ++i) {
    auto &p = cloud_in_out->at(i);
    Eigen::Vector3d point3d(p.x, p.y, p.z);
    point3d = pose * point3d;
    p.x = point3d.x();
    p.y = point3d.y();
    p.z = point3d.z();
  }
}

// @brief extract the indiced points from apoint cloud
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

// @brief get the maxmium and minimium in each axis of a indiced point cloud
// old name: du_get_min_max_3d
template <typename PointCloudT>
void GetMinMaxIn3D(
    const PointCloudT &cloud, const base::PointIndices &indices,
    Eigen::Matrix<typename PointCloudT::PointType::PointType, 4, 1> *min_p,
    Eigen::Matrix<typename PointCloudT::PointType::PointType, 4, 1> *max_p) {
  typedef typename PointCloudT::PointType::PointType Type;
  (*min_p)[0] = (*min_p)[1] = (*min_p)[2] = std::numeric_limits<Type>::max();
  (*max_p)[0] = (*max_p)[1] = (*max_p)[2] = -std::numeric_limits<Type>::max();
  (*min_p)[3] = 0.0;
  (*max_p)[3] = 0.0;
  for (size_t i = 0; i < indices.indices.size(); ++i) {
    const auto &pt = cloud.at(indices.indices[i]);
    if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
      continue;
    }
    (*min_p)[0] = std::min((*min_p)[0], static_cast<Type>(pt.x));
    (*max_p)[0] = std::max((*max_p)[0], static_cast<Type>(pt.x));
    (*min_p)[1] = std::min((*min_p)[1], static_cast<Type>(pt.y));
    (*max_p)[1] = std::max((*max_p)[1], static_cast<Type>(pt.y));
    (*min_p)[2] = std::min((*min_p)[2], static_cast<Type>(pt.z));
    (*max_p)[2] = std::max((*max_p)[2], static_cast<Type>(pt.z));
  }
}

// @brief get the maxmium and minimium in each axis of a point cloud
// old name: du_get_min_max_3d
template <typename PointCloudT>
void GetMinMaxIn3D(
    const PointCloudT &cloud,
    Eigen::Matrix<typename PointCloudT::PointType::PointType, 4, 1> *min_p,
    Eigen::Matrix<typename PointCloudT::PointType::PointType, 4, 1> *max_p) {
  typedef typename PointCloudT::PointType::PointType Type;
  (*min_p)[0] = (*min_p)[1] = (*min_p)[2] = std::numeric_limits<Type>::max();
  (*max_p)[0] = (*max_p)[1] = (*max_p)[2] = -std::numeric_limits<Type>::max();
  (*min_p)[3] = 0.0;
  (*max_p)[3] = 0.0;
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto &pt = cloud.at(i);
    if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
      continue;
    }
    (*min_p)[0] = std::min((*min_p)[0], static_cast<Type>(pt.x));
    (*max_p)[0] = std::max((*max_p)[0], static_cast<Type>(pt.x));
    (*min_p)[1] = std::min((*min_p)[1], static_cast<Type>(pt.y));
    (*max_p)[1] = std::max((*max_p)[1], static_cast<Type>(pt.y));
    (*min_p)[2] = std::min((*min_p)[2], static_cast<Type>(pt.z));
    (*max_p)[2] = std::max((*max_p)[2], static_cast<Type>(pt.z));
  }
}

// @brief get the maxmium and minimium in each axis of a indiced point cloud
// old name: du_get_min_max_3d
template <typename PointCloudT>
void GetMinMaxIn3D(
    const std::shared_ptr<const PointCloudT> cloud,
    const base::PointIndices &indices,
    Eigen::Matrix<typename PointCloudT::PointType::PointType, 4, 1> *min_p,
    Eigen::Matrix<typename PointCloudT::PointType::PointType, 4, 1> *max_p) {
  typedef typename PointCloudT::PointType::PointType Type;
  (*min_p)[0] = (*min_p)[1] = (*min_p)[2] = std::numeric_limits<Type>::max();
  (*max_p)[0] = (*max_p)[1] = (*max_p)[2] = -std::numeric_limits<Type>::max();
  (*min_p)[3] = 0.0;
  (*max_p)[3] = 0.0;
  for (size_t i = 0; i < indices.indices.size(); ++i) {
    const auto &pt = cloud->at(indices.indices[i]);
    if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
      continue;
    }
    (*min_p)[0] = std::min((*min_p)[0], static_cast<Type>(pt.x));
    (*max_p)[0] = std::max((*max_p)[0], static_cast<Type>(pt.x));
    (*min_p)[1] = std::min((*min_p)[1], static_cast<Type>(pt.y));
    (*max_p)[1] = std::max((*max_p)[1], static_cast<Type>(pt.y));
    (*min_p)[2] = std::min((*min_p)[2], static_cast<Type>(pt.z));
    (*max_p)[2] = std::max((*max_p)[2], static_cast<Type>(pt.z));
  }
}

// @brief get the maxmium and minimium in each axis of a point cloud
// old name: du_get_min_max_3d
template <typename PointCloudT>
void GetMinMaxIn3D(
    const std::shared_ptr<const PointCloudT> cloud,
    Eigen::Matrix<typename PointCloudT::PointType::PointType, 4, 1> *min_p,
    Eigen::Matrix<typename PointCloudT::PointType::PointType, 4, 1> *max_p) {
  typedef typename PointCloudT::PointType::PointType Type;
  (*min_p)[0] = (*min_p)[1] = (*min_p)[2] = std::numeric_limits<Type>::max();
  (*max_p)[0] = (*max_p)[1] = (*max_p)[2] = -std::numeric_limits<Type>::max();
  (*min_p)[3] = 0.0;
  (*max_p)[3] = 0.0;
  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto &pt = cloud->at(i);
    if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
      continue;
    }
    (*min_p)[0] = std::min((*min_p)[0], static_cast<Type>(pt.x));
    (*max_p)[0] = std::max((*max_p)[0], static_cast<Type>(pt.x));
    (*min_p)[1] = std::min((*min_p)[1], static_cast<Type>(pt.y));
    (*max_p)[1] = std::max((*max_p)[1], static_cast<Type>(pt.y));
    (*min_p)[2] = std::min((*min_p)[2], static_cast<Type>(pt.z));
    (*max_p)[2] = std::max((*max_p)[2], static_cast<Type>(pt.z));
  }
}

// @brief calculate the centroid of a point cloud
// old name: get_barycenter
template <typename PointCloudT>
Eigen::Matrix<typename PointCloudT::PointType::PointType, 3, 1>
CalculateCentroid(const PointCloudT &cloud) {
  typedef typename PointCloudT::PointType::PointType Type;
  size_t point_num = cloud.size();
  Eigen::Matrix<Type, 3, 1> centroid(0.0, 0.0, 0.0);
  for (const auto &pt : cloud) {
    centroid[0] += pt.x;
    centroid[1] += pt.y;
    centroid[2] += pt.z;
  }
  if (point_num > 0) {
    centroid[0] /= point_num;
    centroid[1] /= point_num;
    centroid[2] /= point_num;
  }
  return centroid;
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
#endif  // PERCEPTION_COMMON_POINT_CLOUD_PROCESSING_COMMON_H_
