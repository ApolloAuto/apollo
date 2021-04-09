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
 * @points_downsampler
 */

#ifndef MODULES_COMMON_UTIL_POINTS_DOWNSAMPLER_H_
#define MODULES_COMMON_UTIL_POINTS_DOWNSAMPLER_H_

#include <cmath>
#include <vector>

#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace common {
namespace util {

/**
 * @brief Calculate the angle between the directions of two points on the path.
 * @param points Points on the path.
 * @param start The index of the first point on the path.
 * @param end The index of the second point on the path.
 * @return The angle between the directions of the start point and the end
 * point.
 */
template <typename Points>
double GetPathAngle(const Points &points, const size_t start,
                    const size_t end) {
  if (start >= static_cast<size_t>(points.size() - 1) ||
      end >= static_cast<size_t>(points.size() - 1)) {
    AERROR << "Input indices are out of the range of the points vector: "
           << "should be less than vector size - 1.";
    return 0.0;
  }
  if (start >= end) {
    AERROR << "Second index must be greater than the first index.";
    return 0.0;
  }
  double vec_start_x = points[start + 1].x() - points[start].x();
  double vec_start_y = points[start + 1].y() - points[start].y();
  double vec_start_norm = std::hypot(vec_start_x, vec_start_y);

  double vec_end_x = points[end + 1].x() - points[end].x();
  double vec_end_y = points[end + 1].y() - points[end].y();
  double vec_end_norm = std::hypot(vec_end_x, vec_end_y);

  double dot_product = vec_start_x * vec_end_x + vec_start_y * vec_end_y;
  double angle = std::acos(dot_product / (vec_start_norm * vec_end_norm));

  return std::isnan(angle) ? 0.0 : angle;
}

/**
 * @brief Downsample the points on the path according to the angle.
 * @param points Points on the path.
 * @param angle_threshold Points are sampled when the accumulated direction
 * change exceeds the threshold.
 * @return sampled_indices Indices of all sampled points, or empty when fail.
 */
template <typename Points>
std::vector<int> DownsampleByAngle(const Points &points,
                                   const double angle_threshold) {
  std::vector<int> sampled_indices;
  if (points.size() == 0) {
    return sampled_indices;
  }

  if (angle_threshold < 0.0) {
    AERROR << "Input angle threshold is negative.";
    return sampled_indices;
  }
  sampled_indices.push_back(0);
  if (points.size() > 1) {
    size_t start = 0;
    size_t end = 1;
    double accum_degree = 0.0;
    while (end < static_cast<size_t>(points.size() - 1)) {
      double angle = GetPathAngle(points, start, end);
      accum_degree += std::fabs(angle);

      if (accum_degree > angle_threshold) {
        sampled_indices.push_back(end);
        start = end;
        accum_degree = 0.0;
      }
      ++end;
    }
    sampled_indices.push_back(end);
  }

  ADEBUG << "Point Vector is downsampled from " << points.size() << " to "
         << sampled_indices.size();

  return sampled_indices;
}

/**
 * @brief Downsample the points on the path based on distance.
 * @param points Points on the path.
 * @param downsampleDistance downsample rate for a normal path
 * @param steepTurnDownsampleDistance downsample rate for a steep turn path
 * @return sampled_indices Indices of all sampled points, or empty when fail.
 */
template <typename Points>
std::vector<int> DownsampleByDistance(const Points &points,
                                      int downsampleDistance,
                                      int steepTurnDownsampleDistance) {
  std::vector<int> sampled_indices;
  if (points.size() <= 4) {
    // No need to downsample if there are not too many points.
    for (size_t i = 0; i < points.size(); ++i) {
      sampled_indices.push_back(i);
    }
    return sampled_indices;
  }

  using apollo::common::math::Vec2d;
  Vec2d v_start =
      Vec2d(points[1].x() - points[0].x(), points[1].y() - points[0].y());
  Vec2d v_end =
      Vec2d(points[points.size() - 1].x() - points[points.size() - 2].x(),
            points[points.size() - 1].y() - points[points.size() - 2].y());
  bool is_steep_turn = v_start.InnerProd(v_end) <= 0;
  int downsampleRate =
      is_steep_turn ? steepTurnDownsampleDistance : downsampleDistance;

  // Make sure the first point is included
  sampled_indices.push_back(0);

  double accum_distance = 0.0;
  for (size_t pos = 1; pos < points.size() - 1; ++pos) {
    Vec2d point_start = Vec2d(points[pos - 1].x(), points[pos - 1].y());
    Vec2d point_end = Vec2d(points[pos].x(), points[pos].y());
    accum_distance += point_start.DistanceTo(point_end);

    if (accum_distance > downsampleRate) {
      sampled_indices.push_back(pos);
      accum_distance = 0.0;
    }
  }

  // Make sure the last point is included
  sampled_indices.push_back(points.size() - 1);
  return sampled_indices;
}

}  // namespace util
}  // namespace common
}  // namespace apollo

#endif  // MODULES_COMMON_UTIL_POINTS_DOWNSAMPLER_H_
