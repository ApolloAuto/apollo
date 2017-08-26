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
  **/

#include "modules/planning/common/speed/st_boundary.h"

#include <algorithm>

#include "modules/common/log.h"

namespace apollo {
namespace planning {

using Vec2d = common::math::Vec2d;

StBoundary::StBoundary(
    const std::vector<std::pair<STPoint, STPoint>>& point_pairs) {
  CHECK(IsValid(point_pairs))
      << "The input point_pairs are NOT valid.\n The point_pairs should "
         "satisfy the following requirements:\n(1) point_pairs.size() >= "
         "2;\n(2) each pair should have same t;\n(3) both points in pair[i + "
         "1] should have larger t than points in pair[i]";

  for (size_t i = 0; i < point_pairs.size(); ++i) {
    lower_points_.emplace_back(point_pairs[i].first);
    upper_points_.emplace_back(point_pairs[i].second);
  }

  for (auto it = lower_points_.begin(); it != lower_points_.end(); ++it) {
    points_.emplace_back(it->t(), it->s());
  }
  for (auto rit = upper_points_.rbegin(); rit != upper_points_.rend(); ++rit) {
    points_.emplace_back(rit->t(), rit->s());
  }

  BuildFromPoints();
  CalculateArea();

  for (const auto& point : lower_points_) {
    min_s_ = std::fmin(min_s_, point.s());
  }
  for (const auto& point : upper_points_) {
    max_s_ = std::fmax(max_s_, point.s());
  }
  min_t_ = std::fmin(lower_points_.front().t(), upper_points_.front().t());
  max_t_ = std::fmax(lower_points_.back().t(), upper_points_.back().t());
}

bool StBoundary::IsValid(
    const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const {
  if (point_pairs.size() < 2) {
    AERROR << "point_pairs.size() must > 2. current point_pairs.size() = "
           << point_pairs.size();
    return false;
  }

  constexpr double kStBoundaryEpsilon = 1e-9;
  constexpr double kMinDeltaT = 1e-6;
  for (size_t i = 0; i < point_pairs.size(); ++i) {
    const auto& curr_lower = point_pairs[i].first;
    const auto& curr_upper = point_pairs[i].second;
    if (curr_upper.s() < curr_lower.s()) {
      return false;
    }

    if (std::fabs(curr_lower.t() - curr_upper.t()) > kStBoundaryEpsilon) {
      return false;
    }

    if (i + 1 != point_pairs.size()) {
      const auto& next_lower = point_pairs[i + 1].first;
      const auto& next_upper = point_pairs[i + 1].second;
      if (std::fmax(curr_lower.t(), curr_upper.t()) + kMinDeltaT >=
          std::fmin(next_lower.t(), next_upper.t())) {
        return false;
      }
    }
  }
  return true;
}

void StBoundary::CalculateArea() {
  for (size_t i = 0; i + 1 < lower_points_.size(); ++i) {
    area_ += (upper_points_[i].y() - lower_points_[i].y()) *
             (lower_points_[i + 1].x() - lower_points_[i].x());
  }
  area_ *= 0.5;
}

bool StBoundary::IsPointInBoundary(const STPoint& st_point) const {
  if (st_point.t() < min_t_) {
    return false;
  }
  if (st_point.t() > max_t_) {
    return false;
  }

  return IsPointIn(st_point);
}

STPoint StBoundary::BottomLeftPoint() const {
  DCHECK(!points_.empty()) << "StBoundary has zero points.";
  return STPoint(lower_points_.front());
}

STPoint StBoundary::BottomRightPoint() const {
  DCHECK(!points_.empty()) << "StBoundary has zero points.";
  return STPoint(lower_points_.back());
}

STPoint StBoundary::TopRightPoint() const {
  DCHECK(!points_.empty()) << "StBoundary has zero points.";
  return STPoint(upper_points_.back());
}

STPoint StBoundary::TopLeftPoint() const {
  DCHECK(!points_.empty()) << "StBoundary has zero points.";
  return STPoint(upper_points_.front());
}

StBoundary StBoundary::ExpandByS(const double s) const {
  if (points_.empty()) {
    AERROR << "The current st_boundary has NO points.";
    return StBoundary();
  }
  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points_.size(); ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points_[i].y() - s, lower_points_[i].x()),
        STPoint(upper_points_[i].y() + s, upper_points_[i].x()));
  }
  return StBoundary(std::move(point_pairs));
}

StBoundary StBoundary::ExpandByT(const double t) const {
  if (points_.empty()) {
    AERROR << "The current st_boundary has NO points.";
    return StBoundary();
  }
  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  point_pairs.emplace_back(
      STPoint(lower_points_.front().y(), lower_points_.front().x() - t),
      STPoint(upper_points_.front().y(), upper_points_.front().x() - t));

  for (size_t i = 0; i < lower_points_.size(); ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points_[i].y(), lower_points_[i].x()),
        STPoint(upper_points_[i].y(), upper_points_[i].x()));
  }
  point_pairs.emplace_back(
      STPoint(lower_points_.back().y(), lower_points_.back().x() + t),
      STPoint(upper_points_.back().y(), upper_points_.back().x() + t));
  return StBoundary(std::move(point_pairs));
}

StBoundary::BoundaryType StBoundary::boundary_type() const {
  return boundary_type_;
}
void StBoundary::SetBoundaryType(const BoundaryType& boundary_type) {
  boundary_type_ = boundary_type;
}

const std::string& StBoundary::id() const { return id_; }

void StBoundary::SetId(const std::string& id) { id_ = id; }

double StBoundary::characteristic_length() const {
  return characteristic_length_;
}

void StBoundary::SetCharacteristicLength(const double characteristic_length) {
  characteristic_length_ = characteristic_length;
}

bool StBoundary::GetUnblockSRange(const double curr_time, double* s_upper,
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

bool StBoundary::GetBoundarySRange(const double curr_time, double* s_upper,
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

double StBoundary::min_s() const { return min_s_; }
double StBoundary::min_t() const { return min_t_; }
double StBoundary::max_s() const { return max_s_; }
double StBoundary::max_t() const { return max_t_; }

}  // namespace planning
}  // namespace apollo
