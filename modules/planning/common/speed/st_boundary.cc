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
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace planning {

using Vec2d = common::math::Vec2d;

namespace {

uint32_t GetIndex(const std::vector<STPoint>& points, const double t) {
  auto comp = [](const double t, const STPoint& p) { return t < p.t(); };
  auto first_gt = std::upper_bound(points.begin(), points.end(), t, comp);
  return std::distance(points.begin(), first_gt) - 1;
}
}

StBoundary::StBoundary(
    const std::vector<std::pair<STPoint, STPoint>>& point_pairs) {
  CHECK(IsValid(point_pairs))
      << "The input point_pairs are NOT valid.\n The point_pairs should "
         "satisfy the following requirements:\n(1) point_pairs.size() >= "
         "2;\n(2) each pair should have same t;\n(3) both points in pair[i + "
         "1] should have larger t than points in pair[i]";

  for (size_t i = 0; i < point_pairs.size(); ++i) {
    // use same t for both points
    const double t = point_pairs[i].first.t();
    lower_points_.emplace_back(point_pairs[i].first.s(), t);
    upper_points_.emplace_back(point_pairs[i].second.s(), t);
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
  min_t_ = lower_points_.front().t();
  max_t_ = lower_points_.back().t();
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
  if (st_point.t() < min_t_ || st_point.t() > max_t_) {
    return false;
  }
  auto index = GetIndex(lower_points_, st_point.t());
  const double check_upper = common::math::CrossProd(
      st_point, upper_points_[index], upper_points_[index + 1]);
  const double check_lower = common::math::CrossProd(
      st_point, lower_points_[index], lower_points_[index + 1]);

  return (check_upper * check_lower <= 0);
}

STPoint StBoundary::BottomLeftPoint() const {
  DCHECK(!lower_points_.empty()) << "StBoundary has zero points.";
  return lower_points_.front();
}

STPoint StBoundary::BottomRightPoint() const {
  DCHECK(!lower_points_.empty()) << "StBoundary has zero points.";
  return lower_points_.back();
}

STPoint StBoundary::TopRightPoint() const {
  DCHECK(!upper_points_.empty()) << "StBoundary has zero points.";
  return upper_points_.back();
}

STPoint StBoundary::TopLeftPoint() const {
  DCHECK(!upper_points_.empty()) << "StBoundary has zero points.";
  return upper_points_.front();
}

StBoundary StBoundary::ExpandByS(const double s) const {
  if (lower_points_.empty()) {
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
  if (lower_points_.empty()) {
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
  CHECK_NOTNULL(s_upper);
  CHECK_NOTNULL(s_lower);

  *s_upper = s_high_limit_;
  *s_lower = 0.0;
  if (curr_time < min_t_ || curr_time > max_t_) {
    return true;
  }

  auto index = GetIndex(lower_points_, curr_time);
  const double r =
      (curr_time - upper_points_[index].t()) /
      (upper_points_.at(index + 1).t() - upper_points_.at(index).t());

  double upper_cross_s =
      upper_points_[index].s() +
      r * (upper_points_[index + 1].s() - upper_points_[index].s());
  double lower_cross_s =
      lower_points_[index].s() +
      r * (lower_points_[index + 1].s() - lower_points_[index].s());

  if (boundary_type_ == BoundaryType::STOP ||
      boundary_type_ == BoundaryType::YIELD ||
      boundary_type_ == BoundaryType::FOLLOW) {
    *s_upper = std::fmin(*s_upper, lower_cross_s);
  } else if (boundary_type_ == BoundaryType::OVERTAKE) {
    *s_lower = std::fmax(*s_lower, upper_cross_s);
  } else {
    AERROR << "boundary_type is not supported. boundary_type: "
           << static_cast<int>(boundary_type_);
    return false;
  }
  return true;
}

bool StBoundary::GetBoundarySRange(const double curr_time, double* s_upper,
                                   double* s_lower) const {
  CHECK_NOTNULL(s_upper);
  CHECK_NOTNULL(s_lower);
  if (curr_time < min_t_ || curr_time > max_t_) {
    return false;
  }

  auto index = GetIndex(lower_points_, curr_time);
  const double r = (curr_time - upper_points_[index].t()) /
                   (upper_points_[index + 1].t() - upper_points_[index].t());

  *s_upper = upper_points_[index].s() +
             r * (upper_points_[index + 1].s() - upper_points_[index].s());
  *s_lower = lower_points_[index].s() +
             r * (lower_points_[index + 1].s() - lower_points_[index].s());

  *s_upper = std::fmin(*s_upper, s_high_limit_);
  *s_lower = std::fmax(*s_lower, 0.0);
  return true;
}

double StBoundary::DistanceS(const STPoint& st_point) const {
  constexpr double kMaxDistance = 1.0e10;
  double s_upper;
  double s_lower;
  if (GetBoundarySRange(st_point.t(), &s_upper, &s_lower)) {
    return kMaxDistance;
  }
  if (st_point.s() < s_lower) {
    return s_lower - st_point.s();
  } else if (st_point.s() > s_upper) {
    return st_point.s() - s_upper;
  } else {
    return 0.0;
  }
}

double StBoundary::min_s() const { return min_s_; }
double StBoundary::min_t() const { return min_t_; }
double StBoundary::max_s() const { return max_s_; }
double StBoundary::max_t() const { return max_t_; }

}  // namespace planning
}  // namespace apollo
