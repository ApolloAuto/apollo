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
 * @file qp_frenet_frame.cc
 **/
#include "modules/planning/tasks/qp_spline_path/qp_frenet_frame.h"

#include <algorithm>
#include <limits>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/macro.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_util.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

using common::SpeedPoint;
using ConstPathObstacleList = std::vector<const PathObstacle*>;

namespace {

constexpr double kEpsilontol = 1e-6;
}

QpFrenetFrame::QpFrenetFrame(const ReferenceLine& reference_line,
                             const ConstPathObstacleList& path_obstacles,
                             const SpeedData& speed_data,
                             const common::FrenetFramePoint& init_frenet_point,
                             const double start_s, const double end_s,
                             const double time_resolution)
    : reference_line_(reference_line),
      path_obstacles_(path_obstacles),
      speed_data_(speed_data),
      vehicle_param_(
          common::VehicleConfigHelper::instance()->GetConfig().vehicle_param()),
      init_frenet_point_(init_frenet_point),
      feasible_longitudinal_upper_bound_(reference_line_.map_path().length()),
      start_s_(start_s),
      end_s_(end_s),
      time_resolution_(time_resolution) {}

bool QpFrenetFrame::Init(const uint32_t num_points) {
  if (num_points < 2) {
    AERROR << "Number of s points [" << num_points
           << "] is too small to evaluate.";
    return false;
  }

  constexpr double kMinDistance = 1.0e-8;
  if (std::fabs(start_s_ - end_s_) < kMinDistance) {
    AERROR << "Not enough s distance. start_s: " << start_s_
           << ", end_s: " << end_s_ << ", kMinDistance: " << kMinDistance;
    return false;
  }

  if (!CalculateDiscretizedVehicleLocation()) {
    AERROR << "Fail to calculate discretized vehicle location!";
    return false;
  }

  const auto inf = std::numeric_limits<double>::infinity();
  const double resolution = (end_s_ - start_s_) / num_points;
  double s = start_s_;
  while (s <= end_s_) {
    evaluated_knots_.push_back(s);
    hdmap_bound_.emplace_back(-inf, inf);
    static_obstacle_bound_.emplace_back(-inf, inf);
    dynamic_obstacle_bound_.emplace_back(-inf, inf);
    s += resolution;
  }

  // initialize calculation here
  CalculateHDMapBound();

  if (!CalculateObstacleBound()) {
    AERROR << "Calculate obstacle bound failed!";
    return false;
  }
  return true;
}

const ReferenceLine& QpFrenetFrame::GetReferenceLine() const {
  return reference_line_;
}

double QpFrenetFrame::feasible_longitudinal_upper_bound() const {
  return feasible_longitudinal_upper_bound_;
}

bool QpFrenetFrame::GetMapBound(const double s,
                                std::pair<double, double>* const bound) const {
  return GetBound(s, hdmap_bound_, bound);
}

bool QpFrenetFrame::GetStaticObstacleBound(
    const double s, std::pair<double, double>* const bound) const {
  return GetBound(s, static_obstacle_bound_, bound);
}

bool QpFrenetFrame::GetDynamicObstacleBound(
    const double s, std::pair<double, double>* const bound) const {
  return GetBound(s, dynamic_obstacle_bound_, bound);
}

bool QpFrenetFrame::CalculateDiscretizedVehicleLocation() {
  double relative_time = 0.0;
  while (relative_time < speed_data_.TotalTime()) {
    SpeedPoint veh_point;
    if (!speed_data_.EvaluateByTime(relative_time, &veh_point)) {
      AERROR << "Fail to get speed point at relative time " << relative_time;
      return false;
    }
    veh_point.set_t(relative_time);
    discretized_vehicle_location_.push_back(std::move(veh_point));
    relative_time += time_resolution_;
  }
  return true;
}

bool QpFrenetFrame::MapDynamicObstacleWithDecision(
    const PathObstacle& path_obstacle) {
  const Obstacle* ptr_obstacle = path_obstacle.Obstacle();
  if (!path_obstacle.HasLateralDecision()) {
    AERROR << "object has no lateral decision";
    return false;
  }
  const auto& decision = path_obstacle.LateralDecision();
  if (!decision.has_nudge()) {
    AWARN << "only support nudge now";
    return true;
  }
  const auto& nudge = decision.nudge();
  double buffer = std::fabs(nudge.distance_l());

  int nudge_side = (nudge.type() == ObjectNudge::RIGHT_NUDGE) ? 1 : -1;

  for (const SpeedPoint& veh_point : discretized_vehicle_location_) {
    double time = veh_point.t();
    common::TrajectoryPoint trajectory_point =
        ptr_obstacle->GetPointAtTime(time);
    common::math::Box2d obs_box =
        ptr_obstacle->GetBoundingBox(trajectory_point);
    // project obs_box on reference line
    std::vector<common::math::Vec2d> corners;
    obs_box.GetAllCorners(&corners);
    std::vector<common::SLPoint> sl_corners;

    for (const auto& corner_xy : corners) {
      common::SLPoint cur_point;
      if (!reference_line_.XYToSL(corner_xy, &cur_point)) {
        AERROR << "Fail to map xy point " << corner_xy.DebugString() << " to "
               << cur_point.ShortDebugString();
        return false;
      }
      // shift box base on buffer
      cur_point.set_l(cur_point.l() - buffer * nudge_side);
      sl_corners.push_back(std::move(cur_point));
    }

    for (uint32_t i = 0; i < sl_corners.size(); ++i) {
      common::SLPoint sl_first = sl_corners[i % sl_corners.size()];
      common::SLPoint sl_second = sl_corners[(i + 1) % sl_corners.size()];
      if (sl_first.s() < sl_second.s()) {
        std::swap(sl_first, sl_second);
      }

      std::pair<double, double> bound = MapLateralConstraint(
          sl_first, sl_second, nudge_side,
          veh_point.s() - vehicle_param_.back_edge_to_center(),
          veh_point.s() + vehicle_param_.front_edge_to_center());

      // update bound map
      double s_resolution = std::fabs(veh_point.v() * time_resolution_);
      double updated_start_s =
          init_frenet_point_.s() + veh_point.s() - s_resolution;
      double updated_end_s =
          init_frenet_point_.s() + veh_point.s() + s_resolution;
      if (updated_end_s > evaluated_knots_.back() ||
          updated_start_s < evaluated_knots_.front()) {
        continue;
      }
      std::pair<uint32_t, uint32_t> update_index_range =
          FindInterval(updated_start_s, updated_end_s);

      for (uint32_t j = update_index_range.first; j < update_index_range.second;
           ++j) {
        dynamic_obstacle_bound_[j].first =
            std::max(bound.first, dynamic_obstacle_bound_[j].first);
        dynamic_obstacle_bound_[j].second =
            std::min(bound.second, dynamic_obstacle_bound_[j].second);
      }
    }
  }
  return true;
}

bool QpFrenetFrame::MapStaticObstacleWithDecision(
    const PathObstacle& path_obstacle) {
  const auto ptr_obstacle = path_obstacle.Obstacle();
  if (!path_obstacle.HasLateralDecision()) {
    AERROR << "obstacle has no lateral decision";
    return false;
  }
  const auto& decision = path_obstacle.LateralDecision();
  if (!decision.has_nudge()) {
    AWARN << "only support nudge decision now";
    return true;
  }
  const auto& nudge = decision.nudge();
  const double buffer = std::fabs(nudge.distance_l());
  if (nudge.type() == ObjectNudge::RIGHT_NUDGE) {
    const auto& box = ptr_obstacle->PerceptionBoundingBox();
    std::vector<common::math::Vec2d> corners;
    box.GetAllCorners(&corners);
    if (!MapPolygon(corners, buffer, -1, &static_obstacle_bound_)) {
      AERROR << "fail to map polygon with id " << path_obstacle.Id()
             << " in qp frenet frame";
      return false;
    }
  } else {
    const auto& box = ptr_obstacle->PerceptionBoundingBox();
    std::vector<common::math::Vec2d> corners;
    box.GetAllCorners(&corners);
    if (!MapPolygon(corners, buffer, 1, &static_obstacle_bound_)) {
      AERROR << "fail to map polygon with id " << path_obstacle.Id()
             << "in qp frenet frame";
      return false;
    }
  }
  return true;
}

bool QpFrenetFrame::MapPolygon(
    const std::vector<common::math::Vec2d>& corners, const double buffer,
    const int nudge_side,
    std::vector<std::pair<double, double>>* const bound_map) {
  std::vector<common::SLPoint> sl_corners;

  for (const auto& corner_xy : corners) {
    common::SLPoint cur_point;
    if (!reference_line_.XYToSL(corner_xy, &cur_point)) {
      AERROR << "Fail to map xy point " << corner_xy.DebugString() << " to "
             << cur_point.DebugString();
      return false;
    }
    // shift box base on buffer
    cur_point.set_l(cur_point.l() + buffer * nudge_side);
    sl_corners.push_back(std::move(cur_point));
  }

  for (uint32_t i = 0; i < sl_corners.size(); ++i) {
    if (!MapLine(sl_corners[i % sl_corners.size()],
                 sl_corners[(i + 1) % sl_corners.size()], nudge_side,
                 bound_map)) {
      AERROR << "Map box line (sl) " << sl_corners[i].DebugString() << "->"
             << sl_corners[i + 1].DebugString();
      return false;
    }
  }
  return true;
}

bool QpFrenetFrame::MapLine(
    const common::SLPoint& start, const common::SLPoint& end,
    const int nudge_side,
    std::vector<std::pair<double, double>>* const constraint) {
  common::SLPoint near_point = (start.s() < end.s() ? start : end);
  common::SLPoint further_point = (start.s() < end.s() ? end : start);
  std::pair<uint32_t, uint32_t> impact_index =
      FindInterval(near_point.s(), further_point.s());

  if ((near_point.s() < start_s_ - vehicle_param_.back_edge_to_center() &&
       further_point.s() < start_s_ - vehicle_param_.back_edge_to_center()) ||
      (near_point.s() > end_s_ + vehicle_param_.front_edge_to_center() &&
       further_point.s() > end_s_ + vehicle_param_.front_edge_to_center())) {
    return true;
  }

  double distance =
      std::max(std::abs(near_point.s() - further_point.s()), 1e-8);

  for (uint32_t i = impact_index.first; i <= impact_index.second; ++i) {
    double weight = std::abs((evaluated_knots_[i] - near_point.s())) / distance;
    weight = std::max(weight, 0.0);
    weight = std::min(weight, 1.0);
    double boundary =
        near_point.l() * (1 - weight) + further_point.l() * weight;

    if (nudge_side < 0) {
      boundary += vehicle_param_.width() / 2;
      (*constraint)[i].first = std::max(boundary, (*constraint)[i].first);
    } else if (nudge_side > 0) {
      boundary -= vehicle_param_.width() / 2;
      (*constraint)[i].second = std::min(boundary, (*constraint)[i].second);
    }

    if ((*constraint)[i].second < (*constraint)[i].first + 0.3) {
      if (i > 0) {
        feasible_longitudinal_upper_bound_ =
            std::min(evaluated_knots_[i - 1] - kEpsilontol,
                     feasible_longitudinal_upper_bound_);
      } else {
        feasible_longitudinal_upper_bound_ = start_s_;
        return true;
      }

      AERROR << "current mapping constraint, sl point impact index "
             << "near_point: " << near_point.DebugString()
             << "further_point: " << further_point.DebugString()
             << "impact_index: " << impact_index << "(*constraint)[" << i << "]"
             << (*constraint)[i];
    }
  }

  return true;
}

std::pair<double, double> QpFrenetFrame::MapLateralConstraint(
    const common::SLPoint& start, const common::SLPoint& end,
    const int nudge_side, const double s_start, const double s_end) {
  double inf = std::numeric_limits<double>::infinity();
  std::pair<double, double> result = std::make_pair(-inf, inf);

  if (start.s() > s_end || end.s() < s_start) {
    return result;
  }
  double s_front = std::max(start.s(), s_start);
  double s_back = std::min(end.s(), s_end);

  double weight_back = 0.0;
  double weight_front = 0.0;

  if (Double::Compare((end.s() - start.s()), 0.0) > 0) {
    weight_back = (s_back - end.s()) / (end.s() - start.s());
    weight_front = (s_front - start.s()) / (end.s() - start.s());
  }

  common::SLPoint front = util::interpolate(start, end, weight_front);
  common::SLPoint back = util::interpolate(start, end, weight_back);

  if (nudge_side > 0) {
    double l = std::min(front.l(), back.l());
    result.second = l;
  } else if (nudge_side < 0) {
    double l = std::max(front.l(), back.l());
    result.first = l;
  }
  return result;
}

std::pair<uint32_t, uint32_t> QpFrenetFrame::FindInterval(
    const double start, const double end) const {
  double new_start = std::max(start - vehicle_param_.front_edge_to_center(),
                              evaluated_knots_.front());
  double new_end = std::min(end + vehicle_param_.back_edge_to_center(),
                            evaluated_knots_.back());
  uint32_t start_index = FindIndex(new_start);
  uint32_t end_index = FindIndex(new_end);

  if (end > evaluated_knots_[end_index] &&
      end_index + 1 != evaluated_knots_.size()) {
    end_index++;
  }

  return std::make_pair(start_index, end_index);
}

void QpFrenetFrame::CalculateHDMapBound() {
  for (uint32_t i = 0; i < hdmap_bound_.size(); ++i) {
    double left_bound = 0.0;
    double right_bound = 0.0;
    bool suc = reference_line_.get_lane_width(evaluated_knots_[i], &left_bound,
                                              &right_bound);
    if (!suc) {
      AWARN << "Extracting lane width failed at s = " << evaluated_knots_[i];
      right_bound = FLAGS_default_reference_line_width / 2;
      left_bound = FLAGS_default_reference_line_width / 2;
    }

    hdmap_bound_[i].first = -right_bound + vehicle_param_.width() / 2;
    hdmap_bound_[i].second = left_bound - vehicle_param_.width() / 2;

    if (hdmap_bound_[i].first >= hdmap_bound_[i].second) {
      AERROR << "HD Map bound at " << evaluated_knots_[i] << " is infeasible ("
             << hdmap_bound_[i].first << ", " << hdmap_bound_[i].second << ") "
             << std::endl;
      AERROR << "vehicle_param_.width = " << vehicle_param_.width();
      AERROR << "left_bound: " << left_bound;
      AERROR << "right_bound: " << right_bound;
      feasible_longitudinal_upper_bound_ =
          std::min(evaluated_knots_[i], feasible_longitudinal_upper_bound_);
      common::SLPoint sl;
      sl.set_s(evaluated_knots_[i]);
      common::math::Vec2d xy;
      reference_line_.SLToXY(sl, &xy);
      AERROR << "evaluated_knot x: " << std::fixed << xy.x()
             << " y: " << xy.y();
    }
  }
}

bool QpFrenetFrame::CalculateObstacleBound() {
  for (const auto ptr_path_obstacle : path_obstacles_) {
    if (!ptr_path_obstacle->HasLateralDecision()) {
      continue;
    }
    if (ptr_path_obstacle->Obstacle()->IsStatic()) {
      if (!MapStaticObstacleWithDecision(*ptr_path_obstacle)) {
        AERROR << "mapping obstacle with id [" << ptr_path_obstacle->Id()
               << "] failed in qp frenet frame.";
        return false;
      }
    } else {
      if (!MapDynamicObstacleWithDecision(*ptr_path_obstacle)) {
        AERROR << "mapping obstacle with id [" << ptr_path_obstacle->Id()
               << "] failed in qp frenet frame.";
        return false;
      }
    }
  }
  return true;
}

bool QpFrenetFrame::GetBound(
    const double s, const std::vector<std::pair<double, double>>& map_bound,
    std::pair<double, double>* const bound) const {
  if (Double::Compare(s, start_s_, 1e-8) < 0 ||
      Double::Compare(s, end_s_, 1e-8) > 0) {
    AERROR << "Evaluate s location " << s
           << ", is out of trajectory frenet frame range (" << start_s_ << ", "
           << end_s_ << ")";
    return false;
  }

  // linear bound interpolation
  uint32_t lower_index = FindIndex(s);
  const double s_low = evaluated_knots_[lower_index];
  const double s_high = evaluated_knots_[lower_index + 1];
  double weight = Double::Compare(s_low, s_high, 1e-8) < 0
                      ? (s - s_low) / (s_high - s_low)
                      : 0.0;

  double low_first = map_bound[lower_index].first;
  double low_second = map_bound[lower_index].second;
  double high_first = map_bound[lower_index + 1].first;
  double high_second = map_bound[lower_index + 1].second;

  // If there is only one infinity in low and high point, then make it equal
  // to
  // the not inf one.
  if (std::isinf(low_first) && !std::isinf(high_first)) {
    low_first = high_first;
  } else if (!std::isinf(low_first) && std::isinf(high_first)) {
    high_first = low_first;
  }

  if (std::isinf(low_second) && !std::isinf(high_second)) {
    low_second = high_second;
  } else if (!std::isinf(low_second) && std::isinf(high_second)) {
    high_second = low_second;
  }

  if (std::isinf(low_first)) {
    bound->first = low_first;
  } else {
    bound->first = low_first * (1 - weight) + high_first * weight;
  }

  if (std::isinf(low_second)) {
    bound->second = low_second * (1 - weight) + high_second * weight;
  }
  return true;
}

uint32_t QpFrenetFrame::FindIndex(const double s) const {
  auto lower_bound =
      std::lower_bound(evaluated_knots_.begin() + 1, evaluated_knots_.end(), s);
  return std::min(
             static_cast<uint32_t>(evaluated_knots_.size() - 1),
             static_cast<uint32_t>(lower_bound - evaluated_knots_.begin())) -
         1;
}

}  // namespace planning
}  // namespace apollo
