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
#include "modules/perception/common/lidar/common/lidar_object_util.h"

#include <algorithm>
#include <limits>

#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/lidar/common/lidar_log.h"

namespace apollo {
namespace perception {
namespace lidar {

using base::Object;
using base::PointCloud;
using base::PointD;
using base::PointF;

static const float kEpsilon = 1e-6;

void GetBoundingBox2d(const std::shared_ptr<Object>& object,
                      PointCloud<PointD>* box, double expand) {
  box->clear();
  box->resize(4);

  Eigen::Vector3d direction = object->direction.cast<double>();
  Eigen::Vector3d odirection(-direction(1), direction(0), 0.0);
  double half_length = object->size(0) * 0.5 + expand;
  double half_width = object->size(1) * 0.5 + expand;

  box->at(0).x = half_length * direction(0) + half_width * odirection(0) +
                 object->center(0);
  box->at(0).y = half_length * direction(1) + half_width * odirection(1) +
                 object->center(1);
  box->at(1).x = -half_length * direction(0) + half_width * odirection(0) +
                 object->center(0);
  box->at(1).y = -half_length * direction(1) + half_width * odirection(1) +
                 object->center(1);
  box->at(2).x = -half_length * direction(0) - half_width * odirection(0) +
                 object->center(0);
  box->at(2).y = -half_length * direction(1) - half_width * odirection(1) +
                 object->center(1);
  box->at(3).x = half_length * direction(0) - half_width * odirection(0) +
                 object->center(0);
  box->at(3).y = half_length * direction(1) - half_width * odirection(1) +
                 object->center(1);
}

void ComputeObjectShapeFromPolygon(std::shared_ptr<Object> object,
                                   bool use_world_cloud) {
  const PointCloud<PointD>& polygon = object->polygon;
  const PointCloud<PointF>& cloud = object->lidar_supplement.cloud;
  const PointCloud<PointD>& world_cloud = object->lidar_supplement.cloud_world;

  if (polygon.empty() || cloud.empty()) {
    AINFO << "Failed to compute box, polygon size: " << polygon.size()
          << " cloud size: " << cloud.size();
    return;
  }

  // note here we assume direction is not changed
  Eigen::Vector2d polygon_xy;
  Eigen::Vector2d projected_polygon_xy;
  Eigen::Vector3d raw_direction = object->direction.cast<double>();
  Eigen::Vector2d direction = raw_direction.head<2>();
  Eigen::Vector2d odirection(-direction(1), direction(0));

  constexpr double kDoubleMax = std::numeric_limits<double>::max();
  Eigen::Vector2d min_polygon_xy(kDoubleMax, kDoubleMax);
  Eigen::Vector2d max_polygon_xy(-kDoubleMax, -kDoubleMax);

  // note we keep this offset to avoid numeric precision issues in world frame
  Eigen::Vector2d offset(object->polygon[0].x, object->polygon[0].y);

  for (const auto& point : object->polygon.points()) {
    polygon_xy << point.x, point.y;
    polygon_xy -= offset;
    projected_polygon_xy(0) = direction.dot(polygon_xy);
    projected_polygon_xy(1) = odirection.dot(polygon_xy);
    min_polygon_xy(0) = std::min(min_polygon_xy(0), projected_polygon_xy(0));
    min_polygon_xy(1) = std::min(min_polygon_xy(1), projected_polygon_xy(1));
    max_polygon_xy(0) = std::max(max_polygon_xy(0), projected_polygon_xy(0));
    max_polygon_xy(1) = std::max(max_polygon_xy(1), projected_polygon_xy(1));
  }

  double min_z = 0.0;
  double max_z = 0.0;
  if (use_world_cloud) {
    min_z = world_cloud[0].z;
    max_z = world_cloud[0].z;
    for (const auto& point : world_cloud.points()) {
      min_z = std::min(min_z, point.z);
      max_z = std::max(max_z, point.z);
    }
  } else {
    min_z = cloud[0].z;
    max_z = cloud[0].z;
    for (const auto& point : cloud.points()) {
      min_z = std::min(min_z, static_cast<double>(point.z));
      max_z = std::max(max_z, static_cast<double>(point.z));
    }
  }

  // update size
  object->size << static_cast<float>(max_polygon_xy(0) - min_polygon_xy(0)),
      static_cast<float>(max_polygon_xy(1) - min_polygon_xy(1)),
      static_cast<float>(max_z - min_z);
  // for safety issues, set a default minmium value
  object->size(0) = std::max(object->size(0), 0.01f);
  object->size(1) = std::max(object->size(1), 0.01f);
  object->size(2) = std::max(object->size(2), 0.01f);
  // update center
  projected_polygon_xy << (min_polygon_xy(0) + max_polygon_xy(0)) * 0.5,
      (min_polygon_xy(1) + max_polygon_xy(1)) * 0.5;
  polygon_xy = projected_polygon_xy(0) * direction +
               projected_polygon_xy(1) * odirection;
  object->center << polygon_xy(0) + offset(0), polygon_xy(1) + offset(1), min_z;
}

void ComputePolygonDirection(
    const Eigen::Vector3d& ref_center,
    const base::ObjectPtr& object,
    Eigen::Vector3f* direction) {
  if (object->polygon.size() == 0 ||
      object->lidar_supplement.cloud.size() < 4u) {
    return;
  }
  size_t max_point_index = 0;
  size_t min_point_index = 0;
  ComputeMinMaxDirectionPoint(
      object, ref_center, &max_point_index, &min_point_index);
  Eigen::Vector3d max_point(
      object->polygon[max_point_index].x,
      object->polygon[max_point_index].y,
      object->polygon[max_point_index].z);
  Eigen::Vector3d min_point(
      object->polygon[min_point_index].x,
      object->polygon[min_point_index].y,
      object->polygon[min_point_index].z);
  Eigen::Vector3d line = max_point - min_point;
  double max_dis = 0;
  bool vertices_in_vision = false;
  size_t visible_max_start_point_index = 0;
  size_t visible_max_end_point_index = 0;
  for (size_t i = min_point_index, count = 0; count < object->polygon.size();
    i = (i + 1) % object->polygon.size(), ++count) {
    size_t j = (i + 1) % object->polygon.size();
    Eigen::Vector3d p_i;
    p_i[0] = object->polygon[i].x;
    p_i[1] = object->polygon[i].y;
    p_i[2] = object->polygon[i].z;
    Eigen::Vector3d p_j;
    p_j[0] = object->polygon[j].x;
    p_j[1] = object->polygon[j].y;
    p_j[2] = object->polygon[j].z;
    Eigen::Vector3d ray;
    if ((i == min_point_index && j == max_point_index) ||
        (i == max_point_index && j == min_point_index)) {
      continue;
    } else {
      if (j != min_point_index && j != max_point_index) {
        ray = p_j - min_point;
      } else {  // j == min_point_index || j == max_point_index
        ray = p_i - min_point;
      }
      if (line[0] * ray[1] - ray[0] * line[1] < kEpsilon) {
        double dist = sqrt((p_j[0] - p_i[0]) * (p_j[0] - p_i[0]) +
                           (p_j[1] - p_i[1]) * (p_j[1] - p_i[1]));
        if (dist > max_dis) {
          max_dis = dist;
          visible_max_start_point_index = i;
          visible_max_end_point_index = j;
        }
        vertices_in_vision = true;
      }
    }
  }
  size_t start_point_index = 0;
  size_t end_point_index = 0;
  if (vertices_in_vision) {
    start_point_index = visible_max_start_point_index;
    end_point_index = visible_max_end_point_index;
  } else {
    start_point_index = min_point_index;
    end_point_index = max_point_index;
  }
  Eigen::Vector3d start_point(
      object->polygon[start_point_index].x,
      object->polygon[start_point_index].y,
      object->polygon[start_point_index].z);
  Eigen::Vector3d end_point(
      object->polygon[end_point_index].x,
      object->polygon[end_point_index].y,
      object->polygon[end_point_index].z);
  Eigen::Vector3d dir = end_point - start_point;
  *direction << static_cast<float>(dir[0]),
                static_cast<float>(dir[1]),
                static_cast<float>(dir[2]);
  direction->normalize();
}

void ComputeMinMaxDirectionPoint(
    const base::ObjectPtr& object,
    const Eigen::Vector3d& ref_center,
    size_t* max_point_index,
    size_t* min_point_index) {
  Eigen::Vector3d p_0;
  p_0[0] = object->polygon[0].x;
  p_0[1] = object->polygon[0].y;
  p_0[2] = object->polygon[0].z;
  Eigen::Vector3d max_point = p_0 - ref_center;
  Eigen::Vector3d min_point = p_0 - ref_center;
  for (size_t i = 1; i < object->polygon.size(); ++i) {
    Eigen::Vector3d p;
    p[0] = object->polygon[i].x;
    p[1] = object->polygon[i].y;
    p[2] = object->polygon[i].z;
    Eigen::Vector3d ray = p - ref_center;
    // clock direction
    if (max_point[0] * ray[1] - ray[0] * max_point[1] < kEpsilon) {
      max_point = ray;
      *max_point_index = i;
    }
    // unclock direction
    if (min_point[0] * ray[1] - ray[0] * min_point[1] > kEpsilon) {
      min_point = ray;
      *min_point_index = i;
    }
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
