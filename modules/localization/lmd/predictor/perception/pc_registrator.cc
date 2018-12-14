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

#include "modules/localization/lmd/predictor/perception/pc_registrator.h"

#include <algorithm>
#include <limits>

#include "modules/common/log.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace localization {

using apollo::common::Point3D;
using apollo::common::PointENU;
using apollo::common::math::LineSegment2d;
using apollo::common::math::RotateAxis;
using apollo::common::math::Vec2d;

namespace {
constexpr double kHeadingOptRange = 1.0;
constexpr double kXOptRange = 3.0;
constexpr double kYOptRange = 3.0;
constexpr int kHeadingOptRatio = 12;
constexpr int kXOptRatio = 12;
constexpr int kYOptRatio = 12;
constexpr int kOptIterNum = 3;
constexpr double kNotFoundError = 1000.0;
constexpr double kMovingXCostRatio = 2.0;
constexpr double kMovingYCostRatio = kMovingXCostRatio;
}  // namespace

PCRegistrator::PCRegistrator(PCMap* map) {
  CHECK_NOTNULL(map);
  map_ = map;
}

void PCRegistrator::Register(const std::vector<PCSourcePoint>& source_points,
                             const PointENU& position_estimated,
                             double heading_estimated, PointENU* position,
                             double* heading) const {
  CHECK(position_estimated.has_x());
  CHECK(position_estimated.has_y());
  CHECK_NOTNULL(position);
  CHECK_NOTNULL(heading);

  auto current_error = std::numeric_limits<double>::max();
  position->CopyFrom(position_estimated);
  *heading = heading_estimated;

  // find the near position and heading func
  auto find_near = [&](double heading_lower, double heading_upper,
                       double heading_step, double x_lower, double x_upper,
                       double x_step, double y_lower, double y_upper,
                       double y_step) {
    for (auto heading_testing = heading_lower; heading_testing <= heading_upper;
         heading_testing += heading_step) {
      for (auto x_testing = x_lower; x_testing <= x_upper;
           x_testing += x_step) {
        for (auto y_testing = y_lower; y_testing <= y_upper;
             y_testing += y_step) {
          PointENU position_testing;
          position_testing.set_x(x_testing);
          position_testing.set_y(y_testing);
          position_testing.set_z(position->z());
          auto error =
              ComputeError(source_points, position_testing, heading_testing);

          double flu_x, flu_y;
          RotateAxis(
              heading_estimated, position_testing.x() - position_estimated.x(),
              position_testing.y() - position_estimated.y(), &flu_x, &flu_y);
          error += std::sqrt(flu_x * flu_x * kMovingXCostRatio +
                             flu_y * flu_y * kMovingYCostRatio);

          if (error < current_error) {
            current_error = error;
            position->CopyFrom(position_testing);
            *heading = heading_testing;
          }
        }
      }
    }
  };

  auto heading_range = kHeadingOptRange;
  auto x_range = kXOptRange;
  auto y_range = kYOptRange;
  auto heading_step_ratio = kHeadingOptRatio;
  auto x_step_ratio = kXOptRatio;
  auto y_step_ratio = kYOptRatio;
  for (auto i = 0; i < kOptIterNum; ++i) {
    auto heading_step = heading_range / heading_step_ratio;
    auto heading_lower = (*heading) - heading_range / 2.0;
    auto heading_upper = (*heading) + heading_range / 2.0;

    auto x_step = x_range / x_step_ratio;
    auto x_lower = position->x() - x_range / 2.0;
    auto x_upper = position->x() + x_range / 2.0;

    auto y_step = y_range / y_step_ratio;
    auto y_lower = position->y() - y_range / 2.0;
    auto y_upper = position->y() + y_range / 2.0;

    find_near(heading_lower, heading_upper, heading_step, x_lower, x_upper,
              x_step, y_lower, y_upper, y_step);

    heading_range = heading_step * 2.0;
    x_range = x_step * 2.0;
    y_range = y_step * 2.0;

    heading_step_ratio -= 2;
    heading_step_ratio = std::max(2, heading_step_ratio);
    x_step_ratio /= 2;
    x_step_ratio = std::max(2, x_step_ratio);
    y_step_ratio /= 2;
    y_step_ratio = std::max(2, y_step_ratio);
  }

  ADEBUG << "On registrating, min error [" << current_error << "]";
}

double PCRegistrator::ComputeError(
    const std::vector<PCSourcePoint>& source_points,
    const apollo::common::PointENU& position, double heading) const {
  double error = 0.0;
  std::size_t not_found = 0;

  PCMapIndex near_ni = 0;
  for (const auto& p : source_points) {
    // FLU to ENU
    double enu_x, enu_y;
    RotateAxis(-heading, p.position.x(), p.position.y(), &enu_x, &enu_y);
    enu_x += position.x();
    enu_y += position.y();

    // find a point from map
    PointENU enu_position;
    enu_position.set_x(enu_x);
    enu_position.set_y(enu_y);
    enu_position.set_z(0.0);
    double nearest_d2;
    PCMapIndex nearest_pi;
    std::tie(near_ni, nearest_pi) =
        map_->GetNearestPointOpt(near_ni, enu_position, &nearest_d2);
    if (nearest_pi == (PCMapIndex)-1) {
      near_ni = 0;
      not_found++;
      error += kNotFoundError;
      continue;
    }
    const auto& nearest_p = map_->Point(nearest_pi);

    // get distance
    auto p0 = Vec2d(nearest_p.position.x(), nearest_p.position.y());
    auto pd = Vec2d(enu_x, enu_y);
    auto d2 = nearest_d2;

    if (nearest_p.prev != (PCMapIndex)-1) {
      const auto& prev_p = map_->Point(nearest_p.prev);
      auto p1 = Vec2d(prev_p.position.x(), prev_p.position.y());
      d2 = std::min(LineSegment2d(p0, p1).DistanceSquareTo(pd), d2);
    }

    if (nearest_p.next != (PCMapIndex)-1) {
      const auto& next_p = map_->Point(nearest_p.next);
      auto p1 = Vec2d(next_p.position.x(), next_p.position.y());
      d2 = std::min(LineSegment2d(p0, p1).DistanceSquareTo(pd), d2);
    }

    // get error
    error += std::sqrt(d2);
  }

  return error;
}

}  // namespace localization
}  // namespace apollo
