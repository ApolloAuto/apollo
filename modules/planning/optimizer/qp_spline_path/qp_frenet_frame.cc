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
#include "modules/planning/optimizer/qp_spline_path/qp_frenet_frame.h"

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
  if (num_points <= 1) {
    AERROR << "Number of s points [" << num_points
           << "] is too small to evaluate.";
    return false;
  }

  const double resolution = (end_s_ - start_s_) / num_points;
  if (Double::Compare(start_s_, end_s_, 1e-8) > 0) {
    AERROR << "Not enough s distance. start_s: " << start_s_
           << ", end_s: " << end_s_;
    return false;
  }

  if (!CalculateDiscretizedVehicleLocation()) {
    AERROR << "Fail to calculate discretized vehicle location!";
    return false;
  }

  const auto inf = std::numeric_limits<double>::infinity();
  double s = start_s_;
  while (s <= end_s_) {
    evaluated_knots_.push_back(std::move(s));
    hdmap_bound_.push_back(std::make_pair(-inf, inf));
    static_obstacle_bound_.push_back(std::make_pair(-inf, inf));
    dynamic_obstacle_bound_.push_back(std::make_pair(-inf, inf));
    s += resolution;
  }

  // initialize calculation here
  if (!CalculateHDMapBound()) {
    AERROR << "Calculate hd map bound failed!";
    return false;
  }
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

bool QpFrenetFrame::GetOverallBound(
    const double s, std::pair<double, double>* const bound) const {
  if (!GetMapBound(s, bound)) {
    return false;
  }

  std::pair<double, double> obs_bound =
      std::make_pair(-std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::infinity());

  if (!GetStaticObstacleBound(s, &obs_bound)) {
    return false;
  }

  bound->first = std::max(obs_bound.first, bound->first);
  bound->second = std::min(obs_bound.second, bound->second);

  if (!GetDynamicObstacleBound(s, &obs_bound)) {
    return false;
  }

  bound->first = std::max(obs_bound.first, bound->first);
  bound->second = std::min(obs_bound.second, bound->second);
  return true;
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

bool QpFrenetFrame::FindLongitudinalDistance(const double time,
                                             SpeedPoint* const speed_point) {
  return speed_data_.get_speed_point_with_time(time, speed_point);
}

bool QpFrenetFrame::CalculateDiscretizedVehicleLocation() {
  double relative_time = 0.0;

  for (; relative_time < speed_data_.total_time();
       relative_time += time_resolution_) {
    SpeedPoint veh_point;
    if (!speed_data_.get_speed_point_with_time(relative_time, &veh_point)) {
      AERROR << "Fail to get speed point at relative time " << relative_time;
      return false;
    }
    veh_point.set_t(relative_time);
    discretized_vehicle_location_.push_back(std::move(veh_point));
  }

  return true;
}

bool QpFrenetFrame::MapDynamicObstacleWithDecision(
    const PathObstacle& path_obstacle) {
  const std::vector<ObjectDecisionType>& decisions = path_obstacle.Decisions();
  const Obstacle* obstacle = path_obstacle.Obstacle();

  for (const auto& decision : decisions) {
    if (!decision.has_nudge()) {
      continue;
    }
    const auto& nudge = decision.nudge();
    double buffer = std::fabs(nudge.distance_l());

    int nudge_side = nudge.type() == ObjectNudge::RIGHT_NUDGE ? 1 : -1;

    for (const SpeedPoint& veh_point : discretized_vehicle_location_) {
      double time = veh_point.t();
      common::TrajectoryPoint trajectory_point = obstacle->GetPointAtTime(time);
      common::math::Vec2d xy_point(trajectory_point.path_point().x(),
                                   trajectory_point.path_point().y());
      common::math::Box2d obs_box = obstacle->GetBoundingBox(trajectory_point);
      // project obs_box on reference line
      std::vector<common::math::Vec2d> corners;
      obs_box.GetAllCorners(&corners);
      std::vector<common::SLPoint> sl_corners;

      for (uint32_t i = 0; i < corners.size(); ++i) {
        common::SLPoint cur_point;
        common::math::Vec2d xy_point(corners[i].x(), corners[i].y());
        if (!reference_line_.get_point_in_frenet_frame(xy_point, &cur_point)) {
          AERROR << "Fail to map xy point " << corners[i].DebugString()
                 << " to " << cur_point.ShortDebugString();

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
        double updatestart_s_ =
            init_frenet_point_.s() + veh_point.s() - s_resolution;
        double updateend_s_ =
            init_frenet_point_.s() + veh_point.s() + s_resolution;
        if (updateend_s_ > evaluated_knots_.back() ||
            updatestart_s_ < evaluated_knots_.front()) {
          continue;
        }
        std::pair<uint32_t, uint32_t> update_index_range =
            FindInterval(updatestart_s_, updateend_s_);

        for (uint32_t j = update_index_range.first;
             j < update_index_range.second; ++j) {
          dynamic_obstacle_bound_[j].first =
              std::max(bound.first, dynamic_obstacle_bound_[j].first);
          dynamic_obstacle_bound_[j].second =
              std::min(bound.second, dynamic_obstacle_bound_[j].second);
        }
      }
    }
  }

  return true;
}

bool QpFrenetFrame::MapStaticObstacleWithDecision(
    const PathObstacle& path_obstacle) {
  const std::vector<ObjectDecisionType>& object_decisions =
      path_obstacle.Decisions();
  const auto* obstacle = path_obstacle.Obstacle();

  for (const auto& decision : object_decisions) {
    if (!decision.has_nudge()) {
      continue;
    }
    const auto& nudge = decision.nudge();
    const double buffer = std::fabs(nudge.distance_l());
    if (nudge.type() == ObjectNudge::RIGHT_NUDGE) {
      const auto& box = obstacle->PerceptionBoundingBox();
      std::vector<common::math::Vec2d> corners;
      box.GetAllCorners(&corners);
      if (!MapPolygon(corners, buffer, -1, &static_obstacle_bound_)) {
        AERROR << "fail to map polygon with id " << path_obstacle.Id()
               << " in qp frenet frame";
        return false;
      }
    } else {
      const auto& box = obstacle->PerceptionBoundingBox();
      std::vector<common::math::Vec2d> corners;
      box.GetAllCorners(&corners);
      if (!MapPolygon(corners, buffer, 1, &static_obstacle_bound_)) {
        AERROR << "fail to map polygon with id " << path_obstacle.Id()
               << "in qp frenet frame";
        return false;
      }
    }
  }

  return true;
}

bool QpFrenetFrame::MapPolygon(
    const std::vector<common::math::Vec2d>& corners, const double buffer,
    const bool nudge_side,
    std::vector<std::pair<double, double>>* const bound_map) {
  std::vector<common::SLPoint> sl_corners;

  for (uint32_t i = 0; i < corners.size(); ++i) {
    common::SLPoint cur_point;
    common::math::Vec2d xy_point(corners[i].x(), corners[i].y());
    if (!reference_line_.get_point_in_frenet_frame(xy_point, &cur_point)) {
      AERROR << "Fail to map xy point " << corners[i].DebugString() << " to "
             << cur_point.DebugString();
      return false;
    }
    // shift box base on buffer
    cur_point.set_l(cur_point.l() - buffer * nudge_side);
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
  double weight = 0.0;

  for (uint32_t i = impact_index.first; i <= impact_index.second; ++i) {
    weight = std::abs((evaluated_knots_[i] - near_point.s())) / distance;
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

bool QpFrenetFrame::CalculateHDMapBound() {
  for (uint32_t i = 0; i < hdmap_bound_.size(); ++i) {
    double left_bound = 0.0;
    double right_bound = 0.0;
    bool suc = reference_line_.get_lane_width(evaluated_knots_[i], &left_bound,
                                              &right_bound);
    if (!suc) {
      left_bound = FLAGS_default_reference_line_width / 2;
      right_bound = FLAGS_default_reference_line_width / 2;
    }

    hdmap_bound_[i].first = -right_bound;
    hdmap_bound_[i].second = left_bound;
  }
  for (uint32_t i = 0; i < hdmap_bound_.size(); ++i) {
    hdmap_bound_[i].first = -4.0;
    hdmap_bound_[i].second = 4.0;
  }
  return true;
}

bool QpFrenetFrame::CalculateObstacleBound() {
  for (const auto path_obstacle : path_obstacles_) {
    if (path_obstacle->Obstacle()->IsStatic()) {
      if (!MapStaticObstacleWithDecision(*path_obstacle)) {
        AERROR << "mapping obstacle with id [" << path_obstacle->Id()
               << "] failed in qp frenet frame.";
        return false;
      }
    } else {
      if (!MapDynamicObstacleWithDecision(*path_obstacle)) {
        AERROR << "mapping obstacle with id [" << path_obstacle->Id()
               << "] failed in qp frenet frame.";
        return false;
      }
    }
  }
  return true;
}

bool QpFrenetFrame::GetBound(
    const double s, const std::vector<std::pair<double, double>>& bound_map,
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
  double weight = (Double::Compare(s_high, s_low, 1e-8) <= 0
                       ? 0
                       : (s - s_low) / (s_high - s_low));

  double low_first = bound_map[lower_index].first;
  double low_second = bound_map[lower_index].second;
  double high_first = bound_map[lower_index + 1].first;
  double high_second = bound_map[lower_index + 1].second;

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

  bound->first = low_first * (1 - weight) + high_first * weight;
  bound->second = low_second * (1 - weight) + high_second * weight;

  return true;
}

uint32_t QpFrenetFrame::FindIndex(const double s) const {
  auto upper_bound =
      std::lower_bound(evaluated_knots_.begin() + 1, evaluated_knots_.end(), s);
  return std::min(
             static_cast<uint32_t>(evaluated_knots_.size() - 1),
             static_cast<uint32_t>(upper_bound - evaluated_knots_.begin())) -
         1;
}

}  // namespace planning
}  // namespace apollo
