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

#ifndef MODULES_COMMON_UTIL_DOWN_SAMPLER_H_
#define MODULES_COMMON_UTIL_DOWN_SAMPLER_H_

#include <cmath>
#include <vector>

#include "modules/common/log.h"

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
template <typename PointType>
double GetPathAngle(const std::vector<PointType> &points, const int start,
                    const int end) {
  if (start >= points.size() - 1 || end >= points.size() - 1) {
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
 * @brief Down sample the points on the path according to the angle.
 * @param points Points on the path.
 * @param angle_threshold Points are sampled when the accumulated direction
 * change exceeds the threshold.
 * @param sampled_indexes Indexes of all sampled points.
 * @return true if down sampling is successful.
 */
template <typename PointType>
bool DownSampleByAngle(const std::vector<PointType> &points,
                       const double angle_threshold,
                       std::vector<int> *sampled_indices) {
  if (angle_threshold < 0.0) {
    AERROR << "Input angle threshold is negative.";
    return false;
  }
  sampled_indices->push_back(0);
  if (points.size() > 1) {
    int start = 0;
    int end = 1;
    double accum_degree = 0.0;
    while (end < points.size() - 1) {
      double angle = GetPathAngle(points, start, end);
      accum_degree += std::fabs(angle);

      if (accum_degree > angle_threshold) {
        sampled_indices->push_back(end);
        start = end;
        accum_degree = 0.0;
      }
      ++end;
    }
    sampled_indices->push_back(end);
  }

  AINFO << "Point Vector is down sampled from " << points.size() << " to "
        << sampled_indices->size();

  return true;
}

}  // namespace util
}  // namespace common
}  // namespace apollo

#endif  // MODULES_COMMON_UTIL_DOWN_SAMPLER_H_
