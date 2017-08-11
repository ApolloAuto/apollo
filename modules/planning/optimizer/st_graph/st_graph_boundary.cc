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

StGraphBoundary::StGraphBoundary(const PathObstacle* path_obstacle)
    : path_obstacle_(path_obstacle) {
  CHECK_NOTNULL(path_obstacle);
}

const PathObstacle* StGraphBoundary::path_obstacle() const {
  return path_obstacle_;
}

StGraphBoundary::StGraphBoundary(const PathObstacle* path_obstacle,
                                 const std::vector<STPoint>& points)
    : Polygon2d(std::vector<Vec2d>(points.begin(), points.end())),
      path_obstacle_(path_obstacle) {
  CHECK_NOTNULL(path_obstacle);
  CHECK_EQ(points.size(), 4)
      << "StGraphBoundary must have exactly four points. Input points size: "
      << points.size();
  for (const auto& point : points) {
    min_s_ = std::fmin(min_s_, point.s());
    min_t_ = std::fmin(min_t_, point.t());
    max_s_ = std::fmax(max_s_, point.s());
    max_t_ = std::fmax(max_t_, point.t());
  }
}

StGraphBoundary::StGraphBoundary(
    const PathObstacle* path_obstacle,
    const std::vector<::apollo::common::math::Vec2d>& points)
    : Polygon2d(points), path_obstacle_(path_obstacle) {
  CHECK_NOTNULL(path_obstacle);
  CHECK_EQ(points.size(), 4)
      << "StGraphBoundary must have exactly four points. Input points size: "
      << points.size();
  for (const auto& point : points) {
    min_s_ = std::fmin(min_s_, point.y());
    min_t_ = std::fmin(min_t_, point.x());
    max_s_ = std::fmax(max_s_, point.y());
    max_t_ = std::fmax(max_t_, point.x());
  }
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
  return boundary_type_;
}

void StGraphBoundary::SetBoundaryType(const BoundaryType& boundary_type) {
  boundary_type_ = boundary_type;
}

const std::string& StGraphBoundary::id() const { return id_; }

void StGraphBoundary::set_id(const std::string& id) { id_ = id; }

double StGraphBoundary::characteristic_length() const {
  return characteristic_length_;
}

void StGraphBoundary::SetCharacteristicLength(
    const double characteristic_length) {
  characteristic_length_ = characteristic_length;
}

bool StGraphBoundary::GetUnblockSRange(const double curr_time, double* s_upper,
                                       double* s_lower) const {
  const common::math::LineSegment2d segment = {Vec2d(curr_time, 0.0),
                                               Vec2d(curr_time, s_high_limit_)};
  *s_upper = s_high_limit_;
  *s_lower = 0.0;

  Vec2d p_s_first;
  Vec2d p_s_second;

  if (!GetOverlap(segment, &p_s_first, &p_s_second)) {
    ADEBUG << "curr_time[" << curr_time
           << "] is out of the coverage scope of the boundary.";
    return false;
  }
  if (boundary_type_ == BoundaryType::STOP ||
      boundary_type_ == BoundaryType::YIELD ||
      boundary_type_ == BoundaryType::FOLLOW) {
    *s_upper = std::fmin(*s_upper, std::fmin(p_s_first.y(), p_s_second.y()));
  } else if (boundary_type_ == BoundaryType::OVERTAKE) {
    // overtake
    *s_lower = std::fmax(*s_lower, std::fmax(p_s_first.y(), p_s_second.y()));
  } else {
    AERROR << "boundary_type is not supported. boundary_type: "
           << static_cast<int>(boundary_type_);
    return false;
  }
  return true;
}

bool StGraphBoundary::GetBoundarySRange(const double curr_time, double* s_upper,
                                        double* s_lower) const {
  const common::math::LineSegment2d segment = {Vec2d(curr_time, 0.0),
                                               Vec2d(curr_time, s_high_limit_)};
  *s_upper = s_high_limit_;
  *s_lower = 0.0;

  Vec2d p_s_first;
  Vec2d p_s_second;
  if (!GetOverlap(segment, &p_s_first, &p_s_second)) {
    ADEBUG << "curr_time[" << curr_time
           << "] is out of the coverage scope of the boundary.";
    return false;
  }
  *s_upper = std::fmin(*s_upper, std::fmax(p_s_first.y(), p_s_second.y()));
  *s_lower = std::fmax(*s_lower, std::fmin(p_s_first.y(), p_s_second.y()));
  return true;
}

void StGraphBoundary::GetBoundaryTimeScope(double* start_t,
                                           double* end_t) const {
  STPoint left;
  STPoint right;
  ExtremePoints(0.0, &left, &right);
  *start_t = left.t();
  *end_t = right.t();
}

double StGraphBoundary::min_s() const { return min_s_; }
double StGraphBoundary::min_t() const { return min_t_; }
double StGraphBoundary::max_s() const { return max_s_; }
double StGraphBoundary::max_t() const { return max_t_; }

}  // namespace planning
}  // namespace apollo
