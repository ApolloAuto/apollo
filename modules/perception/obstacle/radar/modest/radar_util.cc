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

#include "modules/perception/obstacle/radar/modest/radar_util.h"

#include "modules/common/log.h"

namespace apollo {
namespace perception {
void RadarUtil::MockRadarPolygon(const Eigen::Vector3d &center,
                                 const double length, const double width,
                                 const double theta, PolygonDType *polygon) {
  if (polygon == nullptr) {
    AERROR << "polygon is nullptr";
    return;
  }
  Eigen::Matrix2d rotation;
  rotation << cos(theta), -sin(theta), sin(theta), cos(theta);
  Eigen::Vector2d local_poly(0, 0);
  Eigen::Vector2d world_poly;
  polygon->resize(4);
  local_poly(0) = -0.5 * length;
  local_poly(1) = -0.5 * width;
  world_poly = rotation * local_poly;
  polygon->points[0].x = center(0) + world_poly(0);
  polygon->points[0].y = center(1) + world_poly(1);
  polygon->points[0].z = center(2);
  local_poly(0) = -0.5 * length;
  local_poly(1) = +0.5 * width;
  world_poly = rotation * local_poly;
  polygon->points[1].x = center(0) + world_poly(0);
  polygon->points[1].y = center(1) + world_poly(1);
  polygon->points[1].z = center(2);
  local_poly(0) = +0.5 * length;
  local_poly(1) = +0.5 * width;
  world_poly = rotation * local_poly;
  polygon->points[2].x = center(0) + world_poly(0);
  polygon->points[2].y = center(1) + world_poly(1);
  polygon->points[2].z = center(2);
  local_poly(0) = +0.5 * length;
  local_poly(1) = -0.5 * width;
  world_poly = rotation * local_poly;
  polygon->points[3].x = center(0) + world_poly(0);
  polygon->points[3].y = center(1) + world_poly(1);
  polygon->points[3].z = center(2);
}

}  // namespace perception
}  // namespace apollo
