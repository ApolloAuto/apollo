/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/tool/benchmark/lidar/util/object_util.h"
#include <algorithm>
#include <memory>
#include <set>
#include <vector>

namespace apollo {
namespace perception {
namespace benchmark {

bool get_bbox_vertices(const ObjectConstPtr object,
                       std::vector<Eigen::Vector3d>* vertices) {
  if (object == nullptr) {
    return false;
  }
  if (vertices == nullptr) {
    return false;
  }
  vertices->resize(8);
  Eigen::Vector2d pts[4];
  Eigen::Matrix2d rot;
  double cos_theta = cos(object->yaw);
  double sin_theta = sin(object->yaw);
  rot << cos_theta, -sin_theta, sin_theta, cos_theta;
  pts[0](0) = std::max<double>(object->length, 0.01) * 0.5;
  pts[0](1) = std::max<double>(object->width, 0.01) * 0.5;
  pts[1](0) = -pts[0](0);
  pts[1](1) = pts[0](1);
  pts[2](0) = -pts[0](0);
  pts[2](1) = -pts[0](1);
  pts[3](0) = pts[0](0);
  pts[3](1) = -pts[0](1);
  for (unsigned int i = 0; i < 4; ++i) {
    pts[i] = rot * pts[i];
    (*vertices)[i].head(2) = pts[i] + object->center.head(2);
    (*vertices)[i](2) = object->center(2) + object->height;
    (*vertices)[i + 4].head(2) = pts[i] + object->center.head(2);
    (*vertices)[i + 4](2) = object->center(2);
  }
  return true;
}

bool fill_objects_with_point_cloud(std::vector<ObjectPtr>* objects,
                                   const PointCloudConstPtr cloud) {
  if (objects == nullptr || cloud == nullptr) {
    return false;
  }
  struct ObjectUtil {
    ObjectPtr object_ptr;
    double object_square_radius = 0.0;
    double half_length = 0.0;
    double half_width = 0.0;
    double half_height = 0.0;
    std::shared_ptr<Eigen::Matrix2d> rotation;
  };
  std::vector<ObjectUtil> object_utils(objects->size());

  for (std::size_t i = 0; i < objects->size(); ++i) {
    ObjectPtr& object = objects->at(i);
    if (object->cloud == nullptr) {
      object->cloud.reset(new PointCloud);
    } else {
      object->cloud->points.clear();
    }
    if (object->indices == nullptr) {
      object->indices.reset(new PointIndices);
    } else {
      object->indices->indices.clear();
    }
    object_utils[i].object_ptr = object;
    object_utils[i].object_square_radius =
        0.5 * (object->length * object->length + object->width * object->width);
    double sin_theta = std::sin(object->yaw);
    double cos_theta = std::cos(object->yaw);
    object_utils[i].rotation.reset(new Eigen::Matrix2d);
    *object_utils[i].rotation << cos_theta, sin_theta, -sin_theta, cos_theta;
    object_utils[i].half_length = object->length * 0.5;
    object_utils[i].half_width = object->width * 0.5;
    object_utils[i].half_height = object->height * 0.5;
  }

  std::sort(object_utils.begin(), object_utils.end(),
            [](const ObjectUtil& lhs, const ObjectUtil& rhs) {
              return lhs.object_square_radius < rhs.object_square_radius;
            });

  for (std::size_t n = 0; n < cloud->points.size(); ++n) {
    const Point& point = cloud->points[n];
    for (std::size_t i = 0; i < objects->size(); ++i) {
      ObjectPtr& object = object_utils[i].object_ptr;
      double z_minius_center = static_cast<double>(point.z) - object->center(2);
      if (z_minius_center < -2.0 /*0.0 hack the ground compensation*/
          || z_minius_center > object->height) {
        continue;
      }
      Eigen::Vector2d point_minus_center;
      point_minus_center << static_cast<double>(point.x) - object->center(0),
          static_cast<double>(point.y) - object->center(1);
      double radius = point_minus_center(0) * point_minus_center(0) +
                      point_minus_center(1) * point_minus_center(1);
      if (radius > object_utils[i].object_square_radius) {
        continue;
      }
      point_minus_center = *object_utils[i].rotation * point_minus_center;
      if (point_minus_center(0) >= -object_utils[i].half_length &&
          point_minus_center(0) <= object_utils[i].half_length &&
          point_minus_center(1) >= -object_utils[i].half_width &&
          point_minus_center(1) <= object_utils[i].half_width) {
        object->cloud->points.push_back(point);
        object->indices->indices.push_back(static_cast<int>(n));
        break;
      } else {
        continue;
      }
    }
  }
  return true;
}

bool fill_axis_align_box(ObjectPtr object) {
  if (object->cloud == nullptr || object->cloud->size() == 0) {
    return false;
  }
  auto& points = object->cloud->points;
  float min_x = points[0].x;
  float max_x = points[0].x;
  float min_y = points[0].y;
  float max_y = points[0].y;
  float min_z = points[0].z;
  float max_z = points[0].z;
  for (auto& p : points) {
    min_x = std::min(p.x, min_x);
    max_x = std::max(p.x, max_x);
    min_y = std::min(p.y, min_y);
    max_y = std::max(p.y, max_y);
    min_z = std::min(p.z, min_z);
    max_z = std::max(p.z, max_z);
  }
  object->length = static_cast<double>(max_x - min_x);
  object->width = static_cast<double>(max_y - min_y);
  object->height = static_cast<double>(max_z - min_z);
  object->center << static_cast<double>((max_x + min_x) * 0.5f),
      static_cast<double>((max_y + min_y) * 0.5f), static_cast<double>(min_z);
  object->yaw = 0.0;
  return true;
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
