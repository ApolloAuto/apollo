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
#include <numeric>
#include <vector>

#include "Eigen/Dense"

#include "modules/common/util/eigen_defs.h"

namespace apollo {
namespace perception {
namespace algorithm {

template <class CLOUD_IN_TYPE, class CLOUD_OUT_TYPE>
class ConvexHull2D {
 public:
  template <class EigenType>
  using EigenVector = apollo::common::EigenVector<EigenType>;

 public:
  ConvexHull2D() : in_cloud_(nullptr) {
    points_.reserve(1000.0);
    polygon_indices_.reserve(1000.0);
  }
  ~ConvexHull2D() { in_cloud_ = nullptr; }
  // main interface to get polygon from input point cloud
  bool GetConvexHull(const CLOUD_IN_TYPE& in_cloud,
                     CLOUD_OUT_TYPE* out_polygon) {
    SetPoints(in_cloud);
    if (!GetConvexHullMonotoneChain(out_polygon)) {
      return MockConvexHull(out_polygon);
    }
    return true;
  }
  // main interface to get polygon from input point cloud(without ground points)
  bool GetConvexHullWithoutGround(const CLOUD_IN_TYPE& in_cloud,
                                  const float& distance_above_ground_thres,
                                  CLOUD_OUT_TYPE* out_polygon) {
    CLOUD_IN_TYPE in_cloud_without_ground;
    in_cloud_without_ground.reserve(in_cloud.size());
    for (std::size_t id = 0; id < in_cloud.size(); ++id) {
      // compute point_heigh, note std::numeric_limits<float>::max() is the
      // default value
      if (in_cloud.points_height(id) >= distance_above_ground_thres) {
        in_cloud_without_ground.push_back(in_cloud[id]);
      }
    }
    if (in_cloud_without_ground.empty()) {
      return GetConvexHull(in_cloud, out_polygon);
    } else {
      SetPoints(in_cloud_without_ground);
      if (!GetConvexHullMonotoneChain(out_polygon)) {
        return MockConvexHull(out_polygon);
      }
    }
    return true;
  }
  // main interface to get polygon from input point cloud
  // (without ground points and points above the head of self-driving car)
  bool GetConvexHullWithoutGroundAndHead(
      const CLOUD_IN_TYPE& in_cloud, const float& distance_above_ground_thres,
      const float& distance_beneath_head_thres, CLOUD_OUT_TYPE* out_polygon) {
    CLOUD_IN_TYPE in_cloud_without_ground_and_head;
    in_cloud_without_ground_and_head.reserve(in_cloud.size());
    for (std::size_t id = 0; id < in_cloud.size(); ++id) {
      // compute point_heigh, note std::numeric_limits<float>::max() is the
      // default value
      if (in_cloud.points_height(id) == std::numeric_limits<float>::max() ||
          (in_cloud.points_height(id) >= distance_above_ground_thres &&
           in_cloud.points_height(id) <= distance_beneath_head_thres)) {
        in_cloud_without_ground_and_head.push_back(in_cloud[id]);
      }
    }
    if (in_cloud_without_ground_and_head.empty()) {
      return GetConvexHull(in_cloud, out_polygon);
    } else {
      SetPoints(in_cloud_without_ground_and_head);
      if (!GetConvexHullMonotoneChain(out_polygon)) {
        return MockConvexHull(out_polygon);
      }
    }
    return true;
  }

 private:
  // save points in local memory, and transform to double
  void SetPoints(const CLOUD_IN_TYPE& in_cloud);
  // mock a polygon for some degenerate cases
  bool MockConvexHull(CLOUD_OUT_TYPE* out_polygon);
  // compute convex hull using Andrew's monotone chain algorithm
  bool GetConvexHullMonotoneChain(CLOUD_OUT_TYPE* out_polygon);
  // given 3 ordered points, return true if in counter clock wise.
  bool IsCounterClockWise(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                          const Eigen::Vector2d& p3, const double& eps) {
    Eigen::Vector2d p12 = p2 - p1;
    Eigen::Vector2d p13 = p3 - p1;
    return (p12(0) * p13(1) - p12(1) * p13(0) > eps);
  }

 private:
  EigenVector<Eigen::Vector2d> points_;
  std::vector<std::size_t> polygon_indices_;
  const CLOUD_IN_TYPE* in_cloud_;
};

template <class CLOUD_IN_TYPE, class CLOUD_OUT_TYPE>
void ConvexHull2D<CLOUD_IN_TYPE, CLOUD_OUT_TYPE>::SetPoints(
    const CLOUD_IN_TYPE& in_cloud) {
  points_.resize(in_cloud.size());
  for (std::size_t i = 0; i < points_.size(); ++i) {
    points_[i] << in_cloud[i].x, in_cloud[i].y;
  }
  in_cloud_ = &in_cloud;
}

template <class CLOUD_IN_TYPE, class CLOUD_OUT_TYPE>
bool ConvexHull2D<CLOUD_IN_TYPE, CLOUD_OUT_TYPE>::MockConvexHull(
    CLOUD_OUT_TYPE* out_polygon) {
  if (in_cloud_->size() == 0) {
    return false;
  }
  out_polygon->resize(4);
  Eigen::Matrix<double, 3, 1> maxv;
  Eigen::Matrix<double, 3, 1> minv;
  maxv << in_cloud_->at(0).x, in_cloud_->at(0).y, in_cloud_->at(0).z;
  minv = maxv;
  for (std::size_t i = 1; i < in_cloud_->size(); ++i) {
    maxv(0) = std::max<double>(maxv(0), in_cloud_->at(i).x);
    maxv(1) = std::max<double>(maxv(1), in_cloud_->at(i).y);
    maxv(2) = std::max<double>(maxv(2), in_cloud_->at(i).z);

    minv(0) = std::min<double>(minv(0), in_cloud_->at(i).x);
    minv(1) = std::min<double>(minv(1), in_cloud_->at(i).y);
    minv(2) = std::min<double>(minv(2), in_cloud_->at(i).z);
  }

  static const double eps = 1e-3;
  for (std::size_t i = 0; i < 3; ++i) {
    if (maxv(i) - minv(i) < eps) {
      maxv(i) += eps;
      minv(i) -= eps;
    }
  }

  // double NOT NECESSARY to static_cast float
  out_polygon->at(0).x = minv(0);
  out_polygon->at(0).y = minv(1);
  out_polygon->at(0).z = minv(2);

  out_polygon->at(1).x = maxv(0);
  out_polygon->at(1).y = minv(1);
  out_polygon->at(1).z = minv(2);

  out_polygon->at(2).x = maxv(0);
  out_polygon->at(2).y = maxv(1);
  out_polygon->at(2).z = minv(2);

  out_polygon->at(3).x = minv(0);
  out_polygon->at(3).y = maxv(1);
  out_polygon->at(3).z = minv(2);
  return true;
}

template <class CLOUD_IN_TYPE, class CLOUD_OUT_TYPE>
bool ConvexHull2D<CLOUD_IN_TYPE, CLOUD_OUT_TYPE>::GetConvexHullMonotoneChain(
    CLOUD_OUT_TYPE* out_polygon) {
  if (points_.size() < 3) {
    return false;
  }

  std::vector<std::size_t> sorted_indices(points_.size());
  std::iota(sorted_indices.begin(), sorted_indices.end(), 0);

  static const double eps = 1e-9;
  std::sort(sorted_indices.begin(), sorted_indices.end(),
            [&](const std::size_t& lhs, const std::size_t& rhs) {
              double dx = points_[lhs](0) - points_[rhs](0);
              if (std::abs(dx) > eps) {
                return dx < 0.0;
              }
              return points_[lhs](1) < points_[rhs](1);
            });
  int count = 0;
  int last_count = 1;
  polygon_indices_.clear();
  polygon_indices_.reserve(points_.size());

  std::size_t size2 = points_.size() * 2;
  for (std::size_t i = 0; i < size2; ++i) {
    if (i == points_.size()) {
      last_count = count;
    }
    const std::size_t& idx =
        sorted_indices[(i < points_.size()) ? i : (size2 - 1 - i)];
    const auto& point = points_[idx];
    while (count > last_count &&
           !IsCounterClockWise(points_[polygon_indices_[count - 2]],
                               points_[polygon_indices_[count - 1]], point,
                               eps)) {
      polygon_indices_.pop_back();
      --count;
    }
    polygon_indices_.push_back(idx);
    ++count;
  }
  --count;
  polygon_indices_.pop_back();
  if (count < 3) {
    return false;
  }
  out_polygon->clear();
  out_polygon->resize(polygon_indices_.size());
  // double NOT NECESSARY to static_cast float
  double min_z = in_cloud_->at(0).z;
  for (std::size_t id = 0; id < in_cloud_->size(); ++id) {
    min_z = std::min<double>(in_cloud_->at(id).z, min_z);
  }
  for (std::size_t i = 0; i < polygon_indices_.size(); ++i) {
    out_polygon->at(i).x = points_[polygon_indices_[i]](0);
    out_polygon->at(i).y = points_[polygon_indices_[i]](1);
    out_polygon->at(i).z = min_z;
  }
  return true;
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
