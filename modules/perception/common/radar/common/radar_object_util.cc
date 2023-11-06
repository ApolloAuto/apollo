/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/common/radar/common/radar_object_util.h"

#include <algorithm>
#include <limits>

#include "modules/perception/common/base/radar_point_cloud.h"
#include "modules/perception/common/base/point_cloud.h"

namespace apollo {
namespace perception {
namespace radar4d {

using base::Object;
using base::RadarPointCloud;
using base::RadarPointD;
using base::RadarPointF;
using base::PointCloud;
using base::PointD;

void GetBoundingBox2d(const std::shared_ptr<Object>& object,
                      RadarPointCloud<RadarPointD>* box, double expand) {
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
  const RadarPointCloud<RadarPointF>& cloud =
    object->radar4d_supplement.cloud;
  const RadarPointCloud<RadarPointD>& world_cloud =
    object->radar4d_supplement.cloud_world;

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

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
