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
*   @file: obstacle_st_boundary.cc
**/

#include "modules/planning/optimizer/st_graph/st_graph_boundary.h"

#include <algorithm>

#include "modules/common/log.h"

namespace apollo {
namespace planning {

using Vec2d = apollo::common::math::Vec2d;

STGraphBoundary::STGraphBoundary(const std::vector<STPoint>& points)
    : _boundary_type(BoundaryType::UNKNOWN) {
  CHECK_GE(points.size(), 4);
  for (const auto& point : points) {
    points_.emplace_back(point.t(), point.s());
  }
  BuildFromPoints();
}

STGraphBoundary::STGraphBoundary(
    const std::vector<::apollo::common::math::Vec2d>& points)
    : Polygon2d(points), _boundary_type(BoundaryType::UNKNOWN) {
  CHECK_GE(points.size(), 4);
}

bool STGraphBoundary::IsPointInBoundary(
    const STGraphPoint& st_graph_point) const {
  ::apollo::common::math::Vec2d vec2d = {st_graph_point.point().t(),
                                         st_graph_point.point().s()};
  return IsPointIn(vec2d);
}

bool STGraphBoundary::IsPointInBoundary(const STPoint& st_point) const {
  ::apollo::common::math::Vec2d vec2d = {st_point.t(), st_point.s()};
  return IsPointIn(vec2d);
}

const ::apollo::common::math::Vec2d STGraphBoundary::point(
    const uint32_t index) const {
  CHECK_LT(index, points_.size());
  return points_[index];
}

const std::vector<::apollo::common::math::Vec2d>& STGraphBoundary::points()
    const {
  return points_;
}

bool STGraphBoundary::is_empty() const { return points_.empty(); }

STGraphBoundary::BoundaryType STGraphBoundary::boundary_type() const {
  return _boundary_type;
}

void STGraphBoundary::set_boundary_type(const BoundaryType& boundary_type) {
  _boundary_type = boundary_type;
}

uint32_t STGraphBoundary::id() const { return _id; }

void STGraphBoundary::set_id(const uint32_t id) { _id = id; }

double STGraphBoundary::characteristic_length() const {
  return _characteristic_length;
}

void STGraphBoundary::set_characteristic_length(
    const double characteristic_length) {
  _characteristic_length = characteristic_length;
}

bool STGraphBoundary::get_s_boundary_position(const double curr_time,
                                              double* s_upper,
                                              double* s_lower) const {
  const ::apollo::common::math::LineSegment2d segment = {
      Vec2d(curr_time, 0.0), Vec2d(curr_time, _s_high_limit)};
  *s_upper = _s_high_limit;
  *s_lower = 0.0;

  Vec2d p_s_first;
  Vec2d p_s_second;
  if (!GetOverlap(segment, &p_s_first, &p_s_second)) {
    AERROR << "curr_time[" << curr_time
           << "] is out of the coverage scope of the boundary.";
    return false;
  }
  if (_boundary_type == BoundaryType::STOP ||
      _boundary_type == BoundaryType::YIELD ||
      _boundary_type == BoundaryType::FOLLOW ||
      _boundary_type == BoundaryType::UNKNOWN) {
    *s_upper = std::fmin(*s_upper, std::fmin(p_s_first.y(), p_s_second.y()));
  } else {
    // overtake
    *s_lower = std::fmax(*s_lower, std::fmax(p_s_first.y(), p_s_second.y()));
  }
  return true;
}

bool STGraphBoundary::get_boundary_s_range_by_time(const double curr_time,
                                                   double* s_upper,
                                                   double* s_lower) const {
  const ::apollo::common::math::LineSegment2d segment = {
      Vec2d(curr_time, 0.0), Vec2d(curr_time, _s_high_limit)};
  *s_upper = _s_high_limit;
  *s_lower = 0.0;

  Vec2d p_s_first;
  Vec2d p_s_second;
  if (!GetOverlap(segment, &p_s_first, &p_s_second)) {
    AERROR << "curr_time[ " << curr_time
           << "] is out of the coverage scope of the boundary.";
    return false;
  }
  *s_upper = std::fmin(*s_upper, std::fmax(p_s_first.y(), p_s_second.y()));
  *s_lower = std::fmax(*s_lower, std::fmin(p_s_first.y(), p_s_second.y()));
  return true;
}

void STGraphBoundary::get_boundary_time_scope(double* start_t,
                                              double* end_t) const {
  *start_t = std::fmin(points_.front().y(), points_.back().y());
  *end_t = std::fmax(points_.at(1).y(), points_.at(2).y());
}

}  // namespace planning
}  // namespace apollo
