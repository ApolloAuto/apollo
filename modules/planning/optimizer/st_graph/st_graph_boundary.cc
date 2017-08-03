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
  * @file: obstacle_st_boundary.cc
  **/

#include "modules/planning/optimizer/st_graph/st_graph_boundary.h"

#include <algorithm>

#include "modules/common/log.h"

namespace apollo {
namespace planning {

using Vec2d = common::math::Vec2d;

StGraphBoundary::StGraphBoundary(const std::vector<STPoint>& points)
    : Polygon2d(std::vector<Vec2d>(points.begin(), points.end())) {
  CHECK_EQ(points.size(), 4)
      << "StGraphBoundary must have exactly four points. Input points size: "
      << points.size();
}

StGraphBoundary::StGraphBoundary(
    const std::vector<::apollo::common::math::Vec2d>& points)
    : Polygon2d(points) {
  CHECK_EQ(points.size(), 4)
      << "StGraphBoundary must have exactly four points. Input points size: "
      << points.size();
}

bool StGraphBoundary::IsPointInBoundary(
    const StGraphPoint& st_graph_point) const {
  return IsPointInBoundary(st_graph_point.point());
}

bool StGraphBoundary::IsPointInBoundary(const STPoint& st_point) const {
  return IsPointIn(st_point);
}

Vec2d StGraphBoundary::point(const uint32_t index) const {
  CHECK_LT(index, points_.size()) << "Index[" << index << "] is out of range.";
  return points_[index];
}

const std::vector<Vec2d>& StGraphBoundary::points() const { return points_; }

StGraphBoundary::BoundaryType StGraphBoundary::boundary_type() const {
  return _boundary_type;
}

void StGraphBoundary::set_boundary_type(const BoundaryType& boundary_type) {
  _boundary_type = boundary_type;
}

const std::string& StGraphBoundary::id() const { return _id; }

void StGraphBoundary::set_id(const std::string& id) { _id = id; }

double StGraphBoundary::characteristic_length() const {
  return _characteristic_length;
}

void StGraphBoundary::set_characteristic_length(
    const double characteristic_length) {
  _characteristic_length = characteristic_length;
}

bool StGraphBoundary::GetUnblockSRange(const double curr_time, double* s_upper,
                                       double* s_lower) const {
  const common::math::LineSegment2d segment = {Vec2d(curr_time, 0.0),
                                               Vec2d(curr_time, _s_high_limit)};
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
  } else if (_boundary_type == BoundaryType::OVERTAKE) {
    // overtake
    *s_lower = std::fmax(*s_lower, std::fmax(p_s_first.y(), p_s_second.y()));
  } else {
    AERROR << "boundary_type is not supported. boundary_type: "
           << static_cast<int>(_boundary_type);
    return false;
  }
  return true;
}

bool StGraphBoundary::GetBoundarySRange(const double curr_time, double* s_upper,
                                        double* s_lower) const {
  const common::math::LineSegment2d segment = {Vec2d(curr_time, 0.0),
                                               Vec2d(curr_time, _s_high_limit)};
  *s_upper = _s_high_limit;
  *s_lower = 0.0;

  Vec2d p_s_first;
  Vec2d p_s_second;
  if (!GetOverlap(segment, &p_s_first, &p_s_second)) {
    AERROR << "curr_time[" << curr_time
           << "] is out of the coverage scope of the boundary.";
    return false;
  }
  *s_upper = std::fmin(*s_upper, std::fmax(p_s_first.y(), p_s_second.y()));
  *s_lower = std::fmax(*s_lower, std::fmin(p_s_first.y(), p_s_second.y()));
  return true;
}

void StGraphBoundary::get_boundary_time_scope(double* start_t,
                                              double* end_t) const {
  *start_t = std::fmin(points_.front().y(), points_.back().y());
  *end_t = std::fmax(points_.at(1).y(), points_.at(2).y());
}

}  // namespace planning
}  // namespace apollo
