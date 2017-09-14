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

/**
 * @file
 */

#include <cmath>

#include "modules/l3_perception/l3_perception_util.h"

/**
 * @namespace apollo::l3_perception
 * @brief apollo::l3_perception
 */

namespace apollo {
namespace l3_perception {

double GetAngleFromQuaternion(const Quaternion quaternion) {
  double theta =
    std::atan2(2.0 * quaternion.qw() * quaternion.qz() +
                   quaternion.qx() * quaternion.qy(),
               1.0 - 2.0 * (quaternion.qy() * quaternion.qy() +
                   quaternion.qz() * quaternion.qz())) +
                   std::acos(-1.0) / 2.0;
  return theta;
}

void FillPerceptionPolygon(PerceptionObstacle* const perception_obstacle,
                           const double mid_x, const double mid_y, const double mid_z,
                           const double length, const double width, const double height,
                           const double heading) {
  for (int i = 0; i < 8; ++i) {
    perception_obstacle->add_polygon_point();
  }
  
  // axis z
  for (int i = 0; i < 8; ++i) {
    double sign = i%2 == 0 ? 1.0 : -1.0;
    perception_obstacle->mutable_polygon_point(i)->set_z(mid_z + sign * height / 2.0);
  }

  // axis x
  for (int i = 0; i < 8; ++i) {
    if (i == 0 || i == 1) { 
      perception_obstacle->mutable_polygon_point(i)->set_x(mid_x + 
                         length * std::cos(heading) / 2.0 + width * std::sin(heading) / 2.0);
    } else if (i == 2 || i == 3) {
      perception_obstacle->mutable_polygon_point(i)->set_x(mid_x + 
                         length * std::cos(heading) / 2.0 - width * std::sin(heading) / 2.0);
    } else if (i == 4 || i == 5) {
      perception_obstacle->mutable_polygon_point(i)->set_x(mid_x - 
                         length * std::cos(heading) / 2.0 - width * std::sin(heading) / 2.0);
    } else {
      perception_obstacle->mutable_polygon_point(i)->set_x(mid_x - 
                         length * std::cos(heading) / 2.0 + width * std::sin(heading) / 2.0);
    }
  }
  
  // axis y
  for (int i = 0; i < 8; ++i) {
    if (i == 0 || i == 1) { 
      perception_obstacle->mutable_polygon_point(i)->set_y(mid_y + 
                         length * std::sin(heading) / 2.0 - width * std::cos(heading) / 2.0);
    } else if (i == 2 || i == 3) {
      perception_obstacle->mutable_polygon_point(i)->set_y(mid_y + 
                         length * std::sin(heading) / 2.0 + width * std::cos(heading) / 2.0);
    } else if (i == 4 || i == 5) {
      perception_obstacle->mutable_polygon_point(i)->set_y(mid_y - 
                         length * std::sin(heading) / 2.0 + width * std::cos(heading) / 2.0);
    } else {
      perception_obstacle->mutable_polygon_point(i)->set_y(mid_y - 
                         length * std::sin(heading) / 2.0 - width * std::cos(heading) / 2.0);
    }
  }
}

}  // namespace l3_perception
}  // namespace apollo


