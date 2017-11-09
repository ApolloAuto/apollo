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

#ifndef MODULES_PERCEPTION_OBSTACLE_RADAR_RADAR_UTIL_H_
#define MODULES_PERCEPTION_OBSTACLE_RADAR_RADAR_UTIL_H_

#include <fstream>
#include "modules/perception/obstacle/base/types.h"

namespace apollo {
namespace perception {

class RadarUtil {
 public:
  template<typename PointT>
  static bool IsXyPointIn2dXyPolygon(
      const PointT &point, const PolygonDType &polygon) {
    bool in_poly = false;
    double x1, x2, y1, y2;

    int nr_poly_points = static_cast<int>(polygon.points.size());
    // start with the last point to make the check last point<->first point the first one
    double xold = polygon.points[nr_poly_points - 1].x;
    double yold = polygon.points[nr_poly_points - 1].y;
    for (int i = 0; i < nr_poly_points; i++) {
      double xnew = polygon.points[i].x;
      double ynew = polygon.points[i].y;
      if (xnew > xold) {
        x1 = xold;
        x2 = xnew;
        y1 = yold;
        y2 = ynew;
      } else {
        x1 = xnew;
        x2 = xold;
        y1 = ynew;
        y2 = yold;
      }

      if ((x1 < point.x) == (point.x <= x2)
          && (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1)) {
        in_poly = !in_poly;
      }
      xold = xnew;
      yold = ynew;
    }

    return in_poly;
  }

  template<typename PointT>
  static bool IsXyPointInHdmap(const PointT &p,
                               const std::vector<PolygonDType> &polygons) {
    bool in_flag = false;
    for (int j = 0; j < polygons.size(); j++) {
      if (IsXyPointIn2dXyPolygon<PointT>(p, polygons[j])) {
        in_flag = true;
        break;
      }
    }
    return in_flag;
  }

  static void MockRadarPolygon(const Eigen::Vector3d &center, const double length,
                               const double width, const double theta, PolygonDType &polygon) {
    Eigen::Matrix2d rotation;
    rotation << cos(theta), -sin(theta),
        sin(theta), cos(theta);
    Eigen::Vector2d local_poly(0, 0);
    Eigen::Vector2d world_poly;
    polygon.resize(4);
    local_poly(0) = -0.5 * length;
    local_poly(1) = -0.5 * width;
    world_poly = rotation * local_poly;
    polygon.points[0].x = center(0) + world_poly(0);
    polygon.points[0].y = center(1) + world_poly(1);
    polygon.points[0].z = center(2);
    local_poly(0) = -0.5 * length;
    local_poly(1) = +0.5 * width;
    world_poly = rotation * local_poly;
    polygon.points[1].x = center(0) + world_poly(0);
    polygon.points[1].y = center(1) + world_poly(1);
    polygon.points[1].z = center(2);
    local_poly(0) = +0.5 * length;
    local_poly(1) = +0.5 * width;
    world_poly = rotation * local_poly;
    polygon.points[2].x = center(0) + world_poly(0);
    polygon.points[2].y = center(1) + world_poly(1);
    polygon.points[2].z = center(2);
    local_poly(0) = +0.5 * length;
    local_poly(1) = -0.5 * width;
    world_poly = rotation * local_poly;
    polygon.points[3].x = center(0) + world_poly(0);
    polygon.points[3].y = center(1) + world_poly(1);
    polygon.points[3].z = center(2);
  }
};

}//namespace perception
}//namespace apollo

#endif // MODULES_PERCEPTION_OBSTACLE_RADAR_RADAR_UTIL_H_
