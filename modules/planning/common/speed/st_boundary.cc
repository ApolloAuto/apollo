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

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::math::LineSegment2d;
using apollo::common::math::Vec2d;

STBoundary::STBoundary(
    const std::vector<std::pair<STPoint, STPoint>>& point_pairs,
    bool is_accurate_boundary) {
  ACHECK(IsValid(point_pairs)) << "The input point_pairs are NOT valid";

  std::vector<std::pair<STPoint, STPoint>> reduced_pairs(point_pairs);
  if (!is_accurate_boundary) {
    RemoveRedundantPoints(&reduced_pairs);
  }

  for (const auto& item : reduced_pairs) {
    // use same t for both points
    const double t = item.first.t();
    lower_points_.emplace_back(item.first.s(), t);
    upper_points_.emplace_back(item.second.s(), t);
  }

  for (const auto& point : lower_points_) {
    points_.emplace_back(point.t(), point.s());
  }
  for (auto rit = upper_points_.rbegin(); rit != upper_points_.rend(); ++rit) {
    points_.emplace_back(rit->t(), rit->s());
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

  obstacle_road_right_ending_t_ = std::numeric_limits<double>::lowest();
}

STBoundary STBoundary::CreateInstance(
    const std::vector<STPoint>& lower_points,
    const std::vector<STPoint>& upper_points) {
  if (lower_points.size() != upper_points.size() || lower_points.size() < 2) {
    return STBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points.size(); ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points.at(i).s(), lower_points.at(i).t()),
        STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
  }
  return STBoundary(point_pairs);
}

STBoundary STBoundary::CreateInstanceAccurate(
    const std::vector<STPoint>& lower_points,
    const std::vector<STPoint>& upper_points) {
  if (lower_points.size() != upper_points.size() || lower_points.size() < 2) {
    return STBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points.size(); ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points.at(i).s(), lower_points.at(i).t()),
        STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
  }
  return STBoundary(point_pairs, true);
}

std::string STBoundary::TypeName(BoundaryType type) {
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
  AWARN << "Unknown boundary type " << static_cast<int>(type)
        << ", treated as UNKNOWN";
  return "UNKNOWN";
}

bool STBoundary::GetUnblockSRange(const double curr_time, double* s_upper,
                                  double* s_lower) const {
  CHECK_NOTNULL(s_upper);
  CHECK_NOTNULL(s_lower);

  *s_upper = FLAGS_speed_lon_decision_horizon;
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

  if (curr_time > upper_points_[right].t()) {
    return true;
  }

  const double r =
      (left == right
           ? 0.0
           : (curr_time - upper_points_[left].t()) /
                 (upper_points_[right].t() - upper_points_[left].t()));

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
    ADEBUG << "boundary_type is not supported. boundary_type: "
           << static_cast<int>(boundary_type_);
    return false;
  }
  return true;
}

bool STBoundary::GetBoundarySRange(const double curr_time, double* s_upper,
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
  const double r =
      (left == right
           ? 0.0
           : (curr_time - upper_points_[left].t()) /
                 (upper_points_[right].t() - upper_points_[left].t()));

  *s_upper = upper_points_[left].s() +
             r * (upper_points_[right].s() - upper_points_[left].s());
  *s_lower = lower_points_[left].s() +
             r * (lower_points_[right].s() - lower_points_[left].s());

  *s_upper = std::fmin(*s_upper, FLAGS_speed_lon_decision_horizon);
  *s_lower = std::fmax(*s_lower, 0.0);
  return true;
}

bool STBoundary::GetBoundarySlopes(const double curr_time, double* ds_upper,
                                   double* ds_lower) const {
  if (ds_upper == nullptr || ds_lower == nullptr) {
    return false;
  }
  if (curr_time < min_t_ || curr_time > max_t_) {
    return false;
  }

  static constexpr double kTimeIncrement = 0.05;
  double t_prev = curr_time - kTimeIncrement;
  double prev_s_upper = 0.0;
  double prev_s_lower = 0.0;
  bool has_prev = GetBoundarySRange(t_prev, &prev_s_upper, &prev_s_lower);
  double t_next = curr_time + kTimeIncrement;
  double next_s_upper = 0.0;
  double next_s_lower = 0.0;
  bool has_next = GetBoundarySRange(t_next, &next_s_upper, &next_s_lower);
  double curr_s_upper = 0.0;
  double curr_s_lower = 0.0;
  GetBoundarySRange(curr_time, &curr_s_upper, &curr_s_lower);
  if (!has_prev && !has_next) {
    return false;
  }
  if (has_prev && has_next) {
    *ds_upper = ((next_s_upper - curr_s_upper) / kTimeIncrement +
                 (curr_s_upper - prev_s_upper) / kTimeIncrement) *
                0.5;
    *ds_lower = ((next_s_lower - curr_s_lower) / kTimeIncrement +
                 (curr_s_lower - prev_s_lower) / kTimeIncrement) *
                0.5;
    return true;
  }
  if (has_prev) {
    *ds_upper = (curr_s_upper - prev_s_upper) / kTimeIncrement;
    *ds_lower = (curr_s_lower - prev_s_lower) / kTimeIncrement;
  } else {
    *ds_upper = (next_s_upper - curr_s_upper) / kTimeIncrement;
    *ds_lower = (next_s_lower - curr_s_lower) / kTimeIncrement;
  }
  return true;
}

bool STBoundary::IsPointInBoundary(const STPoint& st_point) const {
  if (st_point.t() <= min_t_ || st_point.t() >= max_t_) {
    return false;
  }
  size_t left = 0;
  size_t right = 0;
  if (!GetIndexRange(lower_points_, st_point.t(), &left, &right)) {
    AERROR << "failed to get index range.";
    return false;
  }
  const double check_upper = common::math::CrossProd(
      st_point, upper_points_[left], upper_points_[right]);
  const double check_lower = common::math::CrossProd(
      st_point, lower_points_[left], lower_points_[right]);

  return (check_upper * check_lower < 0);
}

STBoundary STBoundary::ExpandByS(const double s) const {
  if (lower_points_.empty()) {
    return STBoundary();
  }
  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points_.size(); ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points_[i].s() - s, lower_points_[i].t()),
        STPoint(upper_points_[i].s() + s, upper_points_[i].t()));
  }
  return STBoundary(std::move(point_pairs));
}

STBoundary STBoundary::ExpandByT(const double t) const {
  if (lower_points_.empty()) {
    AERROR << "The current st_boundary has NO points.";
    return STBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  const double left_delta_t = lower_points_[1].t() - lower_points_[0].t();
  const double lower_left_delta_s = lower_points_[1].s() - lower_points_[0].s();
  const double upper_left_delta_s = upper_points_[1].s() - upper_points_[0].s();

  point_pairs.emplace_back(
      STPoint(lower_points_[0].s() - t * lower_left_delta_s / left_delta_t,
              lower_points_[0].t() - t),
      STPoint(upper_points_[0].s() - t * upper_left_delta_s / left_delta_t,
              upper_points_.front().t() - t));

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

  point_pairs.emplace_back(STPoint(lower_points_.back().s() +
                                       t * lower_right_delta_s / right_delta_t,
                                   lower_points_.back().t() + t),
                           STPoint(upper_points_.back().s() +
                                       t * upper_right_delta_s / right_delta_t,
                                   upper_points_.back().t() + t));
  point_pairs.back().second.set_s(
      std::fmax(point_pairs.back().second.s(),
                point_pairs.back().first.s() + kMinSEpsilon));

  return STBoundary(std::move(point_pairs));
}

STBoundary::BoundaryType STBoundary::boundary_type() const {
  return boundary_type_;
}
void STBoundary::SetBoundaryType(const BoundaryType& boundary_type) {
  boundary_type_ = boundary_type;
}

const std::string& STBoundary::id() const { return id_; }

void STBoundary::set_id(const std::string& id) { id_ = id; }

double STBoundary::characteristic_length() const {
  return characteristic_length_;
}

void STBoundary::SetCharacteristicLength(const double characteristic_length) {
  characteristic_length_ = characteristic_length;
}

double STBoundary::min_s() const { return min_s_; }
double STBoundary::min_t() const { return min_t_; }
double STBoundary::max_s() const { return max_s_; }
double STBoundary::max_t() const { return max_t_; }

STBoundary STBoundary::CutOffByT(const double t) const {
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
  return CreateInstance(lower_points, upper_points);
}

STPoint STBoundary::upper_left_point() const {
  DCHECK(!upper_points_.empty()) << "StBoundary has zero points.";
  return upper_points_.front();
}

STPoint STBoundary::upper_right_point() const {
  DCHECK(!upper_points_.empty()) << "StBoundary has zero points.";
  return upper_points_.back();
}

STPoint STBoundary::bottom_left_point() const {
  DCHECK(!lower_points_.empty()) << "StBoundary has zero points.";
  return lower_points_.front();
}

STPoint STBoundary::bottom_right_point() const {
  DCHECK(!lower_points_.empty()) << "StBoundary has zero points.";
  return lower_points_.back();
}

void STBoundary::set_upper_left_point(STPoint st_point) {
  upper_left_point_ = std::move(st_point);
}

void STBoundary::set_upper_right_point(STPoint st_point) {
  upper_right_point_ = std::move(st_point);
}

void STBoundary::set_bottom_left_point(STPoint st_point) {
  bottom_left_point_ = std::move(st_point);
}

void STBoundary::set_bottom_right_point(STPoint st_point) {
  bottom_right_point_ = std::move(st_point);
}

///////////////////////////////////////////////////////////////////////////////
// Private functions for internal usage.

bool STBoundary::IsValid(
    const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const {
  if (point_pairs.size() < 2) {
    AERROR << "point_pairs.size() must >= 2, but current point_pairs.size() = "
           << point_pairs.size();
    return false;
  }

  static constexpr double kStBoundaryEpsilon = 1e-9;
  static constexpr double kMinDeltaT = 1e-6;
  for (size_t i = 0; i < point_pairs.size(); ++i) {
    const auto& curr_lower = point_pairs[i].first;
    const auto& curr_upper = point_pairs[i].second;
    if (curr_upper.s() < curr_lower.s()) {
      AERROR << "ST-boundary's upper-s must >= lower-s";
      return false;
    }
    if (std::fabs(curr_lower.t() - curr_upper.t()) > kStBoundaryEpsilon) {
      AERROR << "Points in every ST-point pair should be at the same time.";
      return false;
    }
    if (i + 1 != point_pairs.size()) {
      const auto& next_lower = point_pairs[i + 1].first;
      const auto& next_upper = point_pairs[i + 1].second;
      if (std::fmax(curr_lower.t(), curr_upper.t()) + kMinDeltaT >=
          std::fmin(next_lower.t(), next_upper.t())) {
        AERROR << "Latter points should have larger t:";
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

bool STBoundary::IsPointNear(const common::math::LineSegment2d& seg,
                             const Vec2d& point, const double max_dist) {
  return seg.DistanceSquareTo(point) < max_dist * max_dist;
}

void STBoundary::RemoveRedundantPoints(
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

bool STBoundary::GetIndexRange(const std::vector<STPoint>& points,
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

}  // namespace planning
}  // namespace apollo
