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
 * @file reference_line.cc
 **/

#include "modules/planning/reference_line/reference_line.h"

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "boost/math/tools/minima.hpp"

#include "modules/common/log.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using MapPath = hdmap::Path;
using apollo::common::SLPoint;
using apollo::common::util::DistanceXY;
using apollo::hdmap::InterpolatedIndex;

ReferenceLine::ReferenceLine(
    const std::vector<ReferencePoint>& reference_points)
    : reference_points_(reference_points),
      map_path_(MapPath(std::vector<hdmap::MapPathPoint>(
          reference_points.begin(), reference_points.end()))) {
  CHECK_EQ(map_path_.num_points(), reference_points_.size());
}

ReferenceLine::ReferenceLine(const MapPath& hdmap_path)
    : map_path_(hdmap_path) {
  for (const auto& point : hdmap_path.path_points()) {
    DCHECK(!point.lane_waypoints().empty());
    const auto& lane_waypoint = point.lane_waypoints()[0];
    reference_points_.emplace_back(
        hdmap::MapPathPoint(point, point.heading(), lane_waypoint), 0.0, 0.0);
  }
  CHECK_EQ(map_path_.num_points(), reference_points_.size());
}

bool ReferenceLine::Stitch(const ReferenceLine& other) {
  if (other.reference_points().empty()) {
    AWARN << "The other reference line is empty";
    return true;
  }
  auto first_point = reference_points_.front();
  common::SLPoint first_sl;
  if (!other.XYToSL(first_point, &first_sl)) {
    AWARN << "failed to project the first point to the other reference line";
    return false;
  }
  constexpr double kStitchingError = 2e-2;
  bool first_join = first_sl.s() > 0 && first_sl.s() < other.Length() &&
                    std::fabs(first_sl.l()) < kStitchingError;
  auto last_point = reference_points_.back();
  common::SLPoint last_sl;
  if (!other.XYToSL(last_point, &last_sl)) {
    AWARN << "failed to project the last point to the other reference line";
    return false;
  }
  bool last_join = last_sl.s() > 0 && last_sl.s() < other.Length() &&
                   std::fabs(last_sl.l()) < kStitchingError;
  const auto& other_points = other.reference_points();
  if (!first_join && !last_join) {
    common::SLPoint other_first;
    if (!XYToSL(other_points.front(), &other_first)) {
      AERROR << "Could not project point : "
             << other_points.front().DebugString();
      return false;
    }
    bool other_on_current = other_first.s() >= 0 &&
                            other_first.s() < Length() &&
                            std::fabs(other_first.l()) < kStitchingError;
    if (other_on_current) {
      return true;
    }
    AERROR << "These reference lines are not connected";
    return false;
  }
  const auto& accumulated_s = other.map_path().accumulated_s();
  auto lower = accumulated_s.begin();
  if (first_join) {
    lower = std::lower_bound(accumulated_s.begin(), accumulated_s.end(),
                             first_sl.s());
    std::size_t start_i = std::distance(accumulated_s.begin(), lower);
    reference_points_.insert(reference_points_.begin(), other_points.begin(),
                             other_points.begin() + start_i);
  }
  if (last_join) {
    auto upper = std::upper_bound(lower, accumulated_s.end(), last_sl.s());
    auto end_i = std::distance(accumulated_s.begin(), upper);
    reference_points_.insert(reference_points_.end(),
                             other_points.begin() + end_i, other_points.end());
  }
  map_path_ = MapPath(std::vector<hdmap::MapPathPoint>(
      reference_points_.begin(), reference_points_.end()));
  return true;
}

ReferencePoint ReferenceLine::GetNearestReferencePoint(
    const common::math::Vec2d& xy) const {
  double min_dist = std::numeric_limits<double>::max();
  int min_index = 0;
  for (std::size_t i = 0; i < reference_points_.size(); ++i) {
    const double distance = DistanceXY(xy, reference_points_[i]);
    if (distance < min_dist) {
      min_dist = distance;
      min_index = i;
    }
  }
  return reference_points_[min_index];
}

bool ReferenceLine::Shrink(const common::math::Vec2d& point,
                           double look_backward, double look_forward) {
  common::SLPoint sl;
  if (!XYToSL(point, &sl)) {
    AERROR << "Failed to project point: " << point.DebugString();
    return false;
  }
  const auto& accumulated_s = map_path_.accumulated_s();
  size_t start_index = 0;
  if (sl.s() > look_backward) {
    auto it_lower = std::lower_bound(accumulated_s.begin(), accumulated_s.end(),
                                     sl.s() - look_backward);
    start_index = std::distance(accumulated_s.begin(), it_lower);
  }
  size_t end_index = reference_points_.size();
  if (sl.s() + look_forward < Length()) {
    auto start_it = accumulated_s.begin();
    std::advance(start_it, start_index);
    auto it_higher =
        std::upper_bound(start_it, accumulated_s.end(), sl.s() + look_forward);
    end_index = std::distance(accumulated_s.begin(), it_higher);
  }
  reference_points_.erase(reference_points_.begin() + end_index,
                          reference_points_.end());
  reference_points_.erase(reference_points_.begin(),
                          reference_points_.begin() + start_index);
  if (reference_points_.size() < 2) {
    AERROR << "Too few reference points after shrinking.";
    return false;
  }
  map_path_ = MapPath(std::vector<hdmap::MapPathPoint>(
      reference_points_.begin(), reference_points_.end()));
  return true;
}

ReferencePoint ReferenceLine::GetNearestReferencePoint(const double s) const {
  const auto& accumulated_s = map_path_.accumulated_s();
  if (s < accumulated_s.front() - 1e-2) {
    AWARN << "The requested s " << s << " < 0";
    return reference_points_.front();
  }
  if (s > accumulated_s.back() + 1e-2) {
    AWARN << "The requested s " << s << " > reference line length "
          << accumulated_s.back();
    return reference_points_.back();
  }
  auto it_lower =
      std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
  if (it_lower == accumulated_s.begin()) {
    return reference_points_.front();
  } else {
    auto index = std::distance(accumulated_s.begin(), it_lower);
    if (std::fabs(accumulated_s[index - 1] - s) <
        std::fabs(accumulated_s[index] - s)) {
      return reference_points_[index - 1];
    } else {
      return reference_points_[index];
    }
  }
}

std::size_t ReferenceLine::GetNearestReferenceIndex(const double s) const {
  const auto& accumulated_s = map_path_.accumulated_s();
  if (s < accumulated_s.front() - 1e-2) {
    AWARN << "The requested s " << s << " < 0";
    return 0;
  }
  if (s > accumulated_s.back() + 1e-2) {
    AWARN << "The requested s " << s << " > reference line length "
          << accumulated_s.back();
    return reference_points_.size() - 1;
  }
  auto it_lower =
      std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
  return std::distance(accumulated_s.begin(), it_lower);
}

std::vector<ReferencePoint> ReferenceLine::GetReferencePoints(
    double start_s, double end_s) const {
  if (start_s < 0.0) {
    start_s = 0.0;
  }
  if (end_s > Length()) {
    end_s = Length();
  }
  std::vector<ReferencePoint> ref_points;
  auto start_index = GetNearestReferenceIndex(start_s);
  auto end_index = GetNearestReferenceIndex(end_s);
  if (start_index < end_index) {
    ref_points.assign(reference_points_.begin() + start_index,
                      reference_points_.begin() + end_index);
  }
  return ref_points;
}

ReferencePoint ReferenceLine::GetReferencePoint(const double s) const {
  const auto& accumulated_s = map_path_.accumulated_s();
  if (s < accumulated_s.front() - 1e-2) {
    AWARN << "The requested s " << s << " < 0";
    return reference_points_.front();
  }
  if (s > accumulated_s.back() + 1e-2) {
    AWARN << "The requested s " << s << " > reference line length "
          << accumulated_s.back();
    return reference_points_.back();
  }

  auto interpolate_index = map_path_.GetIndexFromS(s);

  uint32_t index = interpolate_index.id;
  uint32_t next_index = index + 1;
  if (next_index >= reference_points_.size()) {
    next_index = reference_points_.size() - 1;
  }

  const auto& p0 = reference_points_[index];
  const auto& p1 = reference_points_[next_index];

  const double s0 = accumulated_s[index];
  const double s1 = accumulated_s[next_index];
  return InterpolateWithMatchedIndex(p0, s0, p1, s1, interpolate_index);
}

double ReferenceLine::FindMinDistancePoint(const ReferencePoint& p0,
                                           const double s0,
                                           const ReferencePoint& p1,
                                           const double s1, const double x,
                                           const double y) {
  auto func_dist_square = [&p0, &p1, &s0, &s1, &x, &y](const double s) {
    auto p = Interpolate(p0, s0, p1, s1, s);
    double dx = p.x() - x;
    double dy = p.y() - y;
    return dx * dx + dy * dy;
  };

  return ::boost::math::tools::brent_find_minima(func_dist_square, s0, s1, 8)
      .first;
}

ReferencePoint ReferenceLine::GetReferencePoint(const double x,
                                                const double y) const {
  CHECK_GE(reference_points_.size(), 0);

  auto func_distance_square = [](const ReferencePoint& point, const double x,
                                 const double y) {
    double dx = point.x() - x;
    double dy = point.y() - y;
    return dx * dx + dy * dy;
  };

  double d_min = func_distance_square(reference_points_.front(), x, y);
  double index_min = 0;

  for (uint32_t i = 1; i < reference_points_.size(); ++i) {
    double d_temp = func_distance_square(reference_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }

  uint32_t index_start = (index_min == 0 ? index_min : index_min - 1);
  uint32_t index_end =
      (index_min + 1 == reference_points_.size() ? index_min : index_min + 1);

  if (index_start == index_end) {
    return reference_points_[index_start];
  }

  double s0 = map_path_.accumulated_s()[index_start];
  double s1 = map_path_.accumulated_s()[index_end];

  double s = ReferenceLine::FindMinDistancePoint(
      reference_points_[index_start], s0, reference_points_[index_end], s1, x,
      y);

  return Interpolate(reference_points_[index_start], s0,
                     reference_points_[index_end], s1, s);
}

bool ReferenceLine::SLToXY(const SLPoint& sl_point,
                           common::math::Vec2d* const xy_point) const {
  CHECK_NOTNULL(xy_point);
  if (map_path_.num_points() < 2) {
    AERROR << "The reference line has too few points.";
    return false;
  }

  const auto matched_point = GetReferencePoint(sl_point.s());
  const auto angle = common::math::Angle16::from_rad(matched_point.heading());
  xy_point->set_x(matched_point.x() - common::math::sin(angle) * sl_point.l());
  xy_point->set_y(matched_point.y() + common::math::cos(angle) * sl_point.l());
  return true;
}

bool ReferenceLine::XYToSL(const common::math::Vec2d& xy_point,
                           SLPoint* const sl_point) const {
  DCHECK_NOTNULL(sl_point);
  double s = 0.0;
  double l = 0.0;
  if (!map_path_.GetProjection(xy_point, &s, &l)) {
    AERROR << "Can't get nearest point from path.";
    return false;
  }
  sl_point->set_s(s);
  sl_point->set_l(l);
  return true;
}

ReferencePoint ReferenceLine::InterpolateWithMatchedIndex(
    const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
    const double s1, const InterpolatedIndex& index) const {
  if (std::fabs(s0 - s1) < common::math::kMathEpsilon) {
    return p0;
  }
  double s = s0 + index.offset;
  DCHECK_LE(s0 - 1.0e-6, s) << " s: " << s << " is less than s0 :" << s0;
  DCHECK_LE(s, s1 + 1.0e-6) << "s: " << s << " is larger than s1: " << s1;

  auto map_path_point = map_path_.GetSmoothPoint(index);
  const double kappa = common::math::lerp(p0.kappa(), s0, p1.kappa(), s1, s);
  const double dkappa = common::math::lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);

  return ReferencePoint(map_path_point, kappa, dkappa);
}

ReferencePoint ReferenceLine::Interpolate(const ReferencePoint& p0,
                                          const double s0,
                                          const ReferencePoint& p1,
                                          const double s1, const double s) {
  if (std::fabs(s0 - s1) < common::math::kMathEpsilon) {
    return p0;
  }
  DCHECK_LE(s0 - 1.0e-6, s) << " s: " << s << " is less than s0 :" << s0;
  DCHECK_LE(s, s1 + 1.0e-6) << "s: " << s << " is larger than s1: " << s1;

  const double x = common::math::lerp(p0.x(), s0, p1.x(), s1, s);
  const double y = common::math::lerp(p0.y(), s0, p1.y(), s1, s);
  const double heading =
      common::math::slerp(p0.heading(), s0, p1.heading(), s1, s);
  const double kappa = common::math::lerp(p0.kappa(), s0, p1.kappa(), s1, s);
  const double dkappa = common::math::lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);
  std::vector<hdmap::LaneWaypoint> waypoints;
  if (!p0.lane_waypoints().empty() && !p1.lane_waypoints().empty()) {
    const auto& p0_waypoint = p0.lane_waypoints()[0];
    if ((s - s0) + p0_waypoint.s <= p0_waypoint.lane->total_length()) {
      const double lane_s = p0_waypoint.s + s - s0;
      waypoints.emplace_back(p0_waypoint.lane, lane_s);
    }
    const auto& p1_waypoint = p1.lane_waypoints()[0];
    if (p1_waypoint.lane->id().id() != p0_waypoint.lane->id().id() &&
        p1_waypoint.s - (s1 - s) >= 0) {
      const double lane_s = p1_waypoint.s - (s1 - s);
      waypoints.emplace_back(p1_waypoint.lane, lane_s);
    }
    if (waypoints.empty()) {
      const double lane_s = p0_waypoint.s;
      waypoints.emplace_back(p0_waypoint.lane, lane_s);
    }
  }
  return ReferencePoint(hdmap::MapPathPoint({x, y}, heading, waypoints), kappa,
                        dkappa);
}

const std::vector<ReferencePoint>& ReferenceLine::reference_points() const {
  return reference_points_;
}

const MapPath& ReferenceLine::map_path() const { return map_path_; }

bool ReferenceLine::GetLaneWidth(const double s, double* const left_width,
                                 double* const right_width) const {
  if (map_path_.path_points().empty()) {
    return false;
  }
  return map_path_.GetWidth(s, left_width, right_width);
}

void ReferenceLine::GetLaneFromS(
    const double s, std::vector<hdmap::LaneInfoConstPtr>* lanes) const {
  CHECK_NOTNULL(lanes);
  auto ref_point = GetReferencePoint(s);
  std::unordered_set<hdmap::LaneInfoConstPtr> lane_set;
  for (auto& lane_waypoint : ref_point.lane_waypoints()) {
    if (lane_set.find(lane_waypoint.lane) == lane_set.end()) {
      lanes->push_back(lane_waypoint.lane);
      lane_set.insert(lane_waypoint.lane);
    }
  }
}

bool ReferenceLine::IsOnRoad(const common::math::Vec2d& vec2d_point) const {
  common::SLPoint sl_point;
  if (!XYToSL(vec2d_point, &sl_point)) {
    return false;
  }
  return IsOnRoad(sl_point);
}

bool ReferenceLine::IsOnRoad(const SLBoundary& sl_boundary) const {
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > Length()) {
    return false;
  }
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  double left_width = 0.0;
  double right_width = 0.0;
  map_path_.GetWidth(middle_s, &left_width, &right_width);
  return !(sl_boundary.start_l() > left_width ||
           sl_boundary.end_l() < -right_width);
}

bool ReferenceLine::IsBlockRoad(const common::math::Box2d& box2d,
                                double gap) const {
  return map_path_.OverlapWith(box2d, gap);
}

bool ReferenceLine::IsOnRoad(const SLPoint& sl_point) const {
  if (sl_point.s() <= 0 || sl_point.s() > map_path_.length()) {
    return false;
  }
  double left_width = 0.0;
  double right_width = 0.0;

  if (!GetLaneWidth(sl_point.s(), &left_width, &right_width)) {
    return false;
  }

  return !(sl_point.l() < -right_width || sl_point.l() > left_width);
}

// return a rough approximated SLBoundary using box length. It is guaranteed to
// be larger than the accurate SL boundary.
bool ReferenceLine::GetApproximateSLBoundary(
    const common::math::Box2d& box, const double start_s, const double end_s,
    SLBoundary* const sl_boundary) const {
  double s = 0.0;
  double l = 0.0;
  double distance = 0.0;
  if (!map_path_.GetProjectionWithHueristicParams(box.center(), start_s, end_s,
                                                  &s, &l, &distance)) {
    AERROR << "Can't get projection point from path.";
    return false;
  }

  auto projected_point = map_path_.GetSmoothPoint(s);
  auto rotated_box = box;
  rotated_box.RotateFromCenter(-projected_point.heading());

  std::vector<common::math::Vec2d> corners;
  rotated_box.GetAllCorners(&corners);

  double min_s(std::numeric_limits<double>::max());
  double max_s(std::numeric_limits<double>::lowest());
  double min_l(std::numeric_limits<double>::max());
  double max_l(std::numeric_limits<double>::lowest());

  for (const auto& point : corners) {
    // x <--> s, y <--> l
    // because the box is rotated to align the reference line
    min_s = std::fmin(min_s, point.x() - rotated_box.center().x() + s);
    max_s = std::fmax(max_s, point.x() - rotated_box.center().x() + s);
    min_l = std::fmin(min_l, point.y() - rotated_box.center().y() + l);
    max_l = std::fmax(max_l, point.y() - rotated_box.center().y() + l);
  }
  sl_boundary->set_start_s(min_s);
  sl_boundary->set_end_s(max_s);
  sl_boundary->set_start_l(min_l);
  sl_boundary->set_end_l(max_l);
  return true;
}

bool ReferenceLine::GetSLBoundary(const common::math::Box2d& box,
                                  SLBoundary* const sl_boundary) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  std::vector<common::math::Vec2d> corners;
  box.GetAllCorners(&corners);
  for (const auto& point : corners) {
    SLPoint sl_point;
    if (!XYToSL(point, &sl_point)) {
      AERROR << "failed to get projection for point: " << point.DebugString()
             << " on reference line.";
      return false;
    }
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());
    end_l = std::fmax(end_l, sl_point.l());
  }
  sl_boundary->set_start_s(start_s);
  sl_boundary->set_end_s(end_s);
  sl_boundary->set_start_l(start_l);
  sl_boundary->set_end_l(end_l);
  return true;
}

bool ReferenceLine::GetSLBoundary(const hdmap::Polygon& polygon,
                                  SLBoundary* const sl_boundary) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  for (const auto& point : polygon.point()) {
    SLPoint sl_point;
    if (!XYToSL(point, &sl_point)) {
      AERROR << "failed to get projection for point: " << point.DebugString()
             << " on reference line.";
      return false;
    }
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());
    end_l = std::fmax(end_l, sl_point.l());
  }
  sl_boundary->set_start_s(start_s);
  sl_boundary->set_end_s(end_s);
  sl_boundary->set_start_l(start_l);
  sl_boundary->set_end_l(end_l);
  return true;
}

bool ReferenceLine::HasOverlap(const common::math::Box2d& box) const {
  SLBoundary sl_boundary;
  if (!GetSLBoundary(box, &sl_boundary)) {
    AERROR << "Failed to get sl boundary for box " << box.DebugString();
    return false;
  }
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > Length()) {
    return false;
  }
  if (sl_boundary.start_l() * sl_boundary.end_l() < 0) {
    return false;
  }

  double left_width = 0.0;
  double right_width = 0.0;
  const double mid_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  if (mid_s < 0 || mid_s > Length()) {
    ADEBUG << "ref_s out of range:" << mid_s;
    return false;
  }
  if (!map_path_.GetWidth(mid_s, &left_width, &right_width)) {
    AERROR << "failed to get width at s = " << mid_s;
    return false;
  }
  if (sl_boundary.start_l() > 0) {
    return sl_boundary.start_l() < left_width;
  } else {
    return sl_boundary.end_l() > -right_width;
  }
}

std::string ReferenceLine::DebugString() const {
  const auto limit =
      std::min(reference_points_.size(),
               static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));
  return apollo::common::util::StrCat(
      "point num:", reference_points_.size(),
      apollo::common::util::PrintDebugStringIter(
          reference_points_.begin(), reference_points_.begin() + limit, ""));
}

double ReferenceLine::GetSpeedLimitFromS(const double s) const {
  for (const auto& speed_limit : speed_limit_) {
    if (s >= speed_limit.start_s && s <= speed_limit.end_s) {
      return speed_limit.speed_limit;
    }
  }
  const auto& map_path_point = GetReferencePoint(s);
  double speed_limit = FLAGS_planning_upper_speed_limit;
  for (const auto& lane_waypoint : map_path_point.lane_waypoints()) {
    if (lane_waypoint.lane == nullptr) {
      AWARN << "lane_waypoint.lane is nullptr";
      continue;
    }
    speed_limit =
        std::fmin(lane_waypoint.lane->lane().speed_limit(), speed_limit);
  }
  return speed_limit;
}

void ReferenceLine::AddSpeedLimit(const hdmap::SpeedControl& speed_control) {
  SLBoundary sl_boundary;
  if (GetSLBoundary(speed_control.polygon(), &sl_boundary) &&
      IsOnRoad(sl_boundary)) {
    AddSpeedLimit(sl_boundary.start_s(), sl_boundary.end_s(),
                  speed_control.speed_limit());
  }
}

void ReferenceLine::AddSpeedLimit(double start_s, double end_s,
                                  double speed_limit) {
  // assume no overlaps between speed limit regions.
  speed_limit_.emplace_back(start_s, end_s, speed_limit);
}

}  // namespace planning
}  // namespace apollo
