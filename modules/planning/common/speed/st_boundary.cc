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
#include <utility>

#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace planning {

using common::math::LineSegment2d;
using common::math::Vec2d;

StBoundary::StBoundary(
    const std::vector<std::pair<STPoint, STPoint>>& point_pairs) {
  CHECK(IsValid(point_pairs)) << "The input point_pairs are NOT valid";

  std::vector<std::pair<STPoint, STPoint>> reduced_pairs(point_pairs);
  RemoveRedundantPoints(&reduced_pairs);

  for (const auto& item : reduced_pairs) {
    // use same t for both points
    const double t = item.first.t();
    lower_points_.emplace_back(item.first.s(), t);
    upper_points_.emplace_back(item.second.s(), t);
  }

  for (auto it = lower_points_.begin(); it != lower_points_.end(); ++it) {
    points_.emplace_back(it->x(), it->y());
  }
  for (auto rit = upper_points_.rbegin(); rit != upper_points_.rend(); ++rit) {
    points_.emplace_back(rit->x(), rit->y());
  }

  BuildFromPoints();

  for (const auto& point : lower_points_) {
    min_s_ = std::fmin(min_s_, point.s());
  }
  for (const auto& point : upper_points_) {
    max_s_ = std::fmax(max_s_, point.s());
  }
  min_t_ = lower_points_.front().t();
  max_t_ = lower_points_.back().t();
}

bool StBoundary::IsPointNear(const common::math::LineSegment2d& seg,
                             const Vec2d& point, const double max_dist) {
  return seg.DistanceSquareTo(point) < max_dist * max_dist;
}

std::string StBoundary::TypeName(BoundaryType type) {
  if (type == BoundaryType::FOLLOW) {
    return "FOLLOW";
  } else if (type == BoundaryType::KEEP_CLEAR) {
    return "KEEP_CLEAR";
  } else if (type == BoundaryType::OVERTAKE) {
    return "OVERTAKE";
  } else if (type == BoundaryType::STOP) {
    return "STOP";
  } else if (type == BoundaryType::YIELD) {
    return "YIELD";
  } else if (type == BoundaryType::UNKNOWN) {
    return "UNKNOWN";
  }
  AWARN << "Unkown boundary type " << static_cast<int>(type)
        << ", treated as UNKNOWN";
  return "UNKNOWN";
}

void StBoundary::RemoveRedundantPoints(
    std::vector<std::pair<STPoint, STPoint>>* point_pairs) {
  if (!point_pairs || point_pairs->size() <= 2) {
    return;
  }

  const double kMaxDist = 0.1;
  size_t i = 0;
  size_t j = 1;

  while (i < point_pairs->size() && j + 1 < point_pairs->size()) {
    LineSegment2d lower_seg(point_pairs->at(i).first,
                            point_pairs->at(j + 1).first);
    LineSegment2d upper_seg(point_pairs->at(i).second,
                            point_pairs->at(j + 1).second);
    if (!IsPointNear(lower_seg, point_pairs->at(j).first, kMaxDist) ||
        !IsPointNear(upper_seg, point_pairs->at(j).second, kMaxDist)) {
      ++i;
      if (i != j) {
        point_pairs->at(i) = point_pairs->at(j);
      }
    }
    ++j;
  }
  point_pairs->at(++i) = point_pairs->back();
  point_pairs->resize(i + 1);
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
      AERROR << "s is not increasing";
      return false;
    }

    if (std::fabs(curr_lower.t() - curr_upper.t()) > kStBoundaryEpsilon) {
      AERROR << "t diff is larger in each STPoint pair";
      return false;
    }

    if (i + 1 != point_pairs.size()) {
      const auto& next_lower = point_pairs[i + 1].first;
      const auto& next_upper = point_pairs[i + 1].second;
      if (std::fmax(curr_lower.t(), curr_upper.t()) + kMinDeltaT >=
          std::fmin(next_lower.t(), next_upper.t())) {
        AERROR << "t is not increasing";
        AERROR << " curr_lower: " << curr_lower.DebugString();
        AERROR << " curr_upper: " << curr_upper.DebugString();
        AERROR << " next_lower: " << next_lower.DebugString();
        AERROR << " next_upper: " << next_upper.DebugString();
        return false;
      }
    }
  }
  return true;
}

bool StBoundary::IsPointInBoundary(const STPoint& st_point) const {
  if (st_point.t() <= min_t_ || st_point.t() >= max_t_) {
    return false;
  }
  size_t left = 0;
  size_t right = 0;
  if (!GetIndexRange(lower_points_, st_point.t(), &left, &right)) {
    AERROR << "fait to get index range.";
    return false;
  }
  const double check_upper = common::math::CrossProd(
      st_point, upper_points_[left], upper_points_[right]);
  const double check_lower = common::math::CrossProd(
      st_point, lower_points_[left], lower_points_[right]);

  return (check_upper * check_lower < 0);
}

STPoint StBoundary::BottomLeftPoint() const {
  DCHECK(!lower_points_.empty()) << "StBoundary has zero points.";
  return lower_points_.front();
}

STPoint StBoundary::BottomRightPoint() const {
  DCHECK(!lower_points_.empty()) << "StBoundary has zero points.";
  return lower_points_.back();
}

StBoundary StBoundary::ExpandByS(const double s) const {
  if (lower_points_.empty()) {
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

  const double left_delta_t = lower_points_[1].t() - lower_points_[0].t();
  const double lower_left_delta_s = lower_points_[1].s() - lower_points_[0].s();
  const double upper_left_delta_s = upper_points_[1].s() - upper_points_[0].s();

  point_pairs.emplace_back(
      STPoint(lower_points_[0].y() - t * lower_left_delta_s / left_delta_t,
              lower_points_[0].x() - t),
      STPoint(upper_points_[0].y() - t * upper_left_delta_s / left_delta_t,
              upper_points_.front().x() - t));

  const double kMinSEpsilon = 1e-3;
  point_pairs.front().first.set_s(
      std::fmin(point_pairs.front().second.s() - kMinSEpsilon,
                point_pairs.front().first.s()));

  for (size_t i = 0; i < lower_points_.size(); ++i) {
    point_pairs.emplace_back(lower_points_[i], upper_points_[i]);
  }

  size_t length = lower_points_.size();
  DCHECK_GE(length, 2);

  const double right_delta_t =
      lower_points_[length - 1].t() - lower_points_[length - 2].t();
  const double lower_right_delta_s =
      lower_points_[length - 1].s() - lower_points_[length - 2].s();
  const double upper_right_delta_s =
      upper_points_[length - 1].s() - upper_points_[length - 2].s();

  point_pairs.emplace_back(STPoint(lower_points_.back().y() +
                                       t * lower_right_delta_s / right_delta_t,
                                   lower_points_.back().x() + t),
                           STPoint(upper_points_.back().y() +
                                       t * upper_right_delta_s / right_delta_t,
                                   upper_points_.back().x() + t));
  point_pairs.back().second.set_s(
      std::fmax(point_pairs.back().second.s(),
                point_pairs.back().first.s() + kMinSEpsilon));

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

  size_t left = 0;
  size_t right = 0;
  if (!GetIndexRange(lower_points_, curr_time, &left, &right)) {
    AERROR << "Fail to get index range.";
    return false;
  }
  const double r = (curr_time - upper_points_[left].t()) /
                   (upper_points_.at(right).t() - upper_points_.at(left).t());

  double upper_cross_s =
      upper_points_[left].s() +
      r * (upper_points_[right].s() - upper_points_[left].s());
  double lower_cross_s =
      lower_points_[left].s() +
      r * (lower_points_[right].s() - lower_points_[left].s());

  if (boundary_type_ == BoundaryType::STOP ||
      boundary_type_ == BoundaryType::YIELD ||
      boundary_type_ == BoundaryType::FOLLOW) {
    *s_upper = lower_cross_s;
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

  size_t left = 0;
  size_t right = 0;
  if (!GetIndexRange(lower_points_, curr_time, &left, &right)) {
    AERROR << "Fail to get index range.";
    return false;
  }
  const double r = (curr_time - upper_points_[left].t()) /
                   (upper_points_[right].t() - upper_points_[left].t());

  *s_upper = upper_points_[left].s() +
             r * (upper_points_[right].s() - upper_points_[left].s());
  *s_lower = lower_points_[left].s() +
             r * (lower_points_[right].s() - lower_points_[left].s());

  *s_upper = std::fmin(*s_upper, s_high_limit_);
  *s_lower = std::fmax(*s_lower, 0.0);
  return true;
}

double StBoundary::min_s() const { return min_s_; }
double StBoundary::min_t() const { return min_t_; }
double StBoundary::max_s() const { return max_s_; }
double StBoundary::max_t() const { return max_t_; }

bool StBoundary::GetIndexRange(const std::vector<STPoint>& points,
                               const double t, size_t* left,
                               size_t* right) const {
  CHECK_NOTNULL(left);
  CHECK_NOTNULL(right);
  if (t < points.front().t() || t > points.back().t()) {
    AERROR << "t is out of range. t = " << t;
    return false;
  }
  auto comp = [](const STPoint& p, const double t) { return p.t() < t; };
  auto first_ge = std::lower_bound(points.begin(), points.end(), t, comp);
  size_t index = std::distance(points.begin(), first_ge);
  if (index == 0) {
    *left = *right = 0;
  } else if (first_ge == points.end()) {
    *left = *right = points.size() - 1;
  } else {
    *left = index - 1;
    *right = index;
  }
  return true;
}

StBoundary StBoundary::GenerateStBoundary(
    const std::vector<STPoint>& lower_points,
    const std::vector<STPoint>& upper_points) {
  if (lower_points.size() != upper_points.size() || lower_points.size() < 2) {
    return StBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points.size() && i < upper_points.size(); ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points.at(i).s(), lower_points.at(i).t()),
        STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
  }
  return StBoundary(point_pairs);
}

StBoundary StBoundary::CutOffByT(const double t) const {
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;
  for (size_t i = 0; i < lower_points_.size() && i < upper_points_.size();
       ++i) {
    if (lower_points_[i].t() < t) {
      continue;
    }
    lower_points.push_back(lower_points_[i]);
    upper_points.push_back(upper_points_[i]);
  }
  return GenerateStBoundary(lower_points, upper_points);
}

}  // namespace planning
}  // namespace apollo
