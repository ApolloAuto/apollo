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

bool QpFrenetFrame::Init(const ReferenceLine& reference_line,
                         const DecisionData& decision_data,
                         const SpeedData& speed_data,
                         const common::FrenetFramePoint& init_frenet_point,
                         const double start_s, const double end_s,
                         const double time_resolution,
                         const uint32_t num_points) {
  Clear();
  _vehicle_param =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  _speed_profile = &speed_data;
  _reference_line = &reference_line;
  _feasible_longitudinal_upper_bound =
      _reference_line->reference_map_line().length();
  _decision_data = &decision_data;
  _init_frenet_point = init_frenet_point;
  _start_s = start_s;
  _end_s = end_s;
  _time_resolution = time_resolution;
  if (num_points <= 1) {
    AERROR << "Number of s points is too small to evaluate " << num_points;
    return false;
  }

  const double resolution = (_end_s - _start_s) / num_points;
  if (Double::compare(_start_s, _end_s, 1e-8) > 0) {
    AERROR << "Not enough sl distance with start " << _start_s << ", end "
           << _end_s;
    return false;
  }

  if (!CalculateDiscretizedVehicleLocation()) {
    AERROR << "Fail to calculate discretized vehicle location!";
    return false;
  }

  const double inf = std::numeric_limits<double>::infinity();
  double s = _start_s;
  for (uint32_t i = 0; i <= num_points && s <= _end_s; ++i) {
    _evaluated_knots.push_back(std::move(s));
    _hdmap_bound.push_back(std::make_pair(-inf, inf));
    _static_obstacle_bound.push_back(std::make_pair(-inf, inf));
    _dynamic_obstacle_bound.push_back(std::make_pair(-inf, inf));
    s = _start_s + (i + 1) * resolution;
  }

  // initialize calculation here
  if (!CalculateHDMapBound()) {
    AERROR << "Calculate hd map bound failed!";
    return false;
  }
  if (!CalculateStaticObstacleBound()) {
    AERROR << "Calculate static obstacle bound failed!";
    return false;
  }
  if (!CalculateDynamicObstacleBound()) {
    AERROR << "Calculate dynamic obstacle bound failed!";
    return false;
  }

  return true;
}

const ReferenceLine* QpFrenetFrame::reference_line() const {
  return _reference_line;
}

double QpFrenetFrame::feasible_longitudinal_upper_bound() const {
  return _feasible_longitudinal_upper_bound;
}

bool QpFrenetFrame::get_overall_bound(
    const double s, std::pair<double, double>* const bound) const {
  if (!get_map_bound(s, bound)) {
    return false;
  }

  std::pair<double, double> obs_bound =
      std::make_pair(-std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::infinity());

  if (!get_static_obstacle_bound(s, &obs_bound)) {
    return false;
  }

  bound->first = std::max(obs_bound.first, bound->first);
  bound->second = std::min(obs_bound.second, bound->second);

  if (!get_dynamic_obstacle_bound(s, &obs_bound)) {
    return false;
  }

  bound->first = std::max(obs_bound.first, bound->first);
  bound->second = std::min(obs_bound.second, bound->second);
  return true;
}

bool QpFrenetFrame::get_map_bound(
    const double s, std::pair<double, double>* const bound) const {
  return GetBound(s, _hdmap_bound, bound);
}

bool QpFrenetFrame::get_static_obstacle_bound(
    const double s, std::pair<double, double>* const bound) const {
  return GetBound(s, _static_obstacle_bound, bound);
}

bool QpFrenetFrame::get_dynamic_obstacle_bound(
    const double s, std::pair<double, double>* const bound) const {
  return GetBound(s, _dynamic_obstacle_bound, bound);
}

bool QpFrenetFrame::FindLongitudinalDistance(const double time,
                                             SpeedPoint* const speed_point) {
  if (_speed_profile == nullptr) {
    return false;
  }

  if (!_speed_profile->get_speed_point_with_time(time, speed_point)) {
    return false;
  }

  return true;
}

bool QpFrenetFrame::CalculateDiscretizedVehicleLocation() {
  double relative_time = 0.0;

  for (; relative_time < _speed_profile->total_time();
       relative_time += _time_resolution) {
    SpeedPoint veh_point;
    if (!_speed_profile->get_speed_point_with_time(relative_time, &veh_point)) {
      AERROR << "Fail to get speed point at relative time " << relative_time;
      return false;
    }
    veh_point.set_t(relative_time);
    _discretized_veh_loc.push_back(std::move(veh_point));
  }

  return true;
}

bool QpFrenetFrame::MapDynamicObstacleWithDecision(const Obstacle& obstacle) {
  const std::vector<ObjectDecisionType>& decision = obstacle.Decisions();

  for (uint32_t i = 0; i < decision.size(); ++i) {
    if (!decision[i].has_nudge()) {
      continue;
    }
    const auto& nudge = decision[i].nudge();
    double buffer = std::fabs(nudge.distance_l());

    int nudge_side = nudge.type() == ObjectNudge::RIGHT_NUDGE ? 1 : -1;

    for (const SpeedPoint& veh_point : _discretized_veh_loc) {
      double time = veh_point.t();
      common::TrajectoryPoint trajectory_point = obstacle.GetPointAtTime(time);
      common::math::Vec2d xy_point(trajectory_point.path_point().x(),
                                   trajectory_point.path_point().y());
      common::math::Box2d obs_box = obstacle.GetBoundingBox(trajectory_point);
      // project obs_box on reference line
      std::vector<common::math::Vec2d> corners;
      obs_box.GetAllCorners(&corners);
      std::vector<common::SLPoint> sl_corners;

      for (uint32_t i = 0; i < corners.size(); ++i) {
        common::SLPoint cur_point;
        common::math::Vec2d xy_point(corners[i].x(), corners[i].y());
        if (!_reference_line->get_point_in_frenet_frame(xy_point, &cur_point)) {
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
            veh_point.s() - _vehicle_param.back_edge_to_center(),
            veh_point.s() + _vehicle_param.front_edge_to_center());

        // update bound map
        double s_resolution = std::fabs(veh_point.v() * _time_resolution);
        double update_start_s =
            _init_frenet_point.s() + veh_point.s() - s_resolution;
        double update_end_s =
            _init_frenet_point.s() + veh_point.s() + s_resolution;
        if (update_end_s > _evaluated_knots.back() ||
            update_start_s < _evaluated_knots.front()) {
          continue;
        }
        std::pair<uint32_t, uint32_t> update_index_range =
            FindInterval(update_start_s, update_end_s);

        for (uint32_t j = update_index_range.first;
             j < update_index_range.second; ++j) {
          _dynamic_obstacle_bound[j].first =
              std::max(bound.first, _dynamic_obstacle_bound[j].first);
          _dynamic_obstacle_bound[j].second =
              std::min(bound.second, _dynamic_obstacle_bound[j].second);
        }
      }
    }
  }

  return true;
}

bool QpFrenetFrame::MapStaticObstacleWithDecision(const Obstacle& obstacle) {
  const std::vector<ObjectDecisionType>& object_decisions =
      obstacle.Decisions();

  for (const auto& decision : object_decisions) {
    if (!decision.has_nudge()) {
      continue;
    }
    const auto& nudge = decision.nudge();
    const double buffer = std::fabs(nudge.distance_l());
    if (nudge.type() == ObjectNudge::RIGHT_NUDGE) {
      const auto& box = obstacle.PerceptionBoundingBox();
      std::vector<common::math::Vec2d> corners;
      box.GetAllCorners(&corners);
      if (!MapPolygon(corners, buffer, -1, &_static_obstacle_bound)) {
        AERROR << "fail to map polygon with id " << obstacle.Id()
               << " in qp frenet frame";
        return false;
      }
    } else {
      const auto& box = obstacle.PerceptionBoundingBox();
      std::vector<common::math::Vec2d> corners;
      box.GetAllCorners(&corners);
      if (!MapPolygon(corners, buffer, 1, &_static_obstacle_bound)) {
        AERROR << "fail to map polygon with id " << obstacle.Id()
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
    if (!_reference_line->get_point_in_frenet_frame(xy_point, &cur_point)) {
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

  if ((near_point.s() < _start_s - _vehicle_param.back_edge_to_center() &&
       further_point.s() < _start_s - _vehicle_param.back_edge_to_center()) ||
      (near_point.s() > _end_s + _vehicle_param.front_edge_to_center() &&
       further_point.s() > _end_s + _vehicle_param.front_edge_to_center())) {
    return true;
  }

  double distance =
      std::max(std::abs(near_point.s() - further_point.s()), 1e-8);
  double weight = 0.0;

  for (uint32_t i = impact_index.first; i <= impact_index.second; ++i) {
    weight = std::abs((_evaluated_knots[i] - near_point.s())) / distance;
    weight = std::max(weight, 0.0);
    weight = std::min(weight, 1.0);
    double boundary =
        near_point.l() * (1 - weight) + further_point.l() * weight;

    if (nudge_side < 0) {
      boundary += _vehicle_param.width() / 2;
      (*constraint)[i].first = std::max(boundary, (*constraint)[i].first);
    } else if (nudge_side > 0) {
      boundary -= _vehicle_param.width() / 2;
      (*constraint)[i].second = std::min(boundary, (*constraint)[i].second);
    }

    if ((*constraint)[i].second < (*constraint)[i].first + 0.3) {
      if (i > 0) {
        _feasible_longitudinal_upper_bound =
            std::min(_evaluated_knots[i - 1] - kEpsilontol,
                     _feasible_longitudinal_upper_bound);
      } else {
        _feasible_longitudinal_upper_bound = _start_s;
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

  if (Double::compare((end.s() - start.s()), 0.0) > 0) {
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
  double new_start = std::max(start - _vehicle_param.front_edge_to_center(),
                              _evaluated_knots.front());
  double new_end = std::min(end + _vehicle_param.back_edge_to_center(),
                            _evaluated_knots.back());
  uint32_t start_index = FindIndex(new_start);
  uint32_t end_index = FindIndex(new_end);

  if (end > _evaluated_knots[end_index] &&
      end_index + 1 != _evaluated_knots.size()) {
    end_index++;
  }

  return std::make_pair(start_index, end_index);
}

bool QpFrenetFrame::CalculateHDMapBound() {
  for (uint32_t i = 0; i < _hdmap_bound.size(); ++i) {
    double left_bound = 0.0;
    double right_bound = 0.0;
    bool suc = _reference_line->get_lane_width(_evaluated_knots[i], &left_bound,
                                               &right_bound);
    if (!suc) {
      left_bound = FLAGS_default_reference_line_width / 2;
      right_bound = FLAGS_default_reference_line_width / 2;
    }

    _hdmap_bound[i].first = -right_bound;
    _hdmap_bound[i].second = left_bound;
  }
  for (uint32_t i = 0; i < _hdmap_bound.size(); ++i) {
    _hdmap_bound[i].first = -4.0;
    _hdmap_bound[i].second = 4.0;
  }
  return true;
}

bool QpFrenetFrame::CalculateStaticObstacleBound() {
  const std::vector<Obstacle*> static_obs_list =
      _decision_data->StaticObstacles();

  for (uint32_t i = 0; i < static_obs_list.size(); ++i) {
    const Obstacle* cur_obstacle = static_obs_list[i];
    if (!MapStaticObstacleWithDecision(*cur_obstacle)) {
      AERROR << "mapping obstacle with id [" << cur_obstacle->Id()
             << "] failed in qp frenet frame.";
      return false;
    }
  }
  return true;
}

bool QpFrenetFrame::CalculateDynamicObstacleBound() {
  const std::vector<Obstacle*> dynamic_obs_list =
      _decision_data->DynamicObstacles();

  for (uint32_t i = 0; i < dynamic_obs_list.size(); ++i) {
    const Obstacle* cur_obstacle = dynamic_obs_list[i];
    if (!MapDynamicObstacleWithDecision(*cur_obstacle)) {
      AERROR << "mapping obstacle with id [" << cur_obstacle->Id()
             << "] failed in qp frenet frame.";
      return false;
    }
  }
  return true;
}

bool QpFrenetFrame::GetBound(
    const double s, const std::vector<std::pair<double, double>>& bound_map,
    std::pair<double, double>* const bound) const {
  if (Double::compare(s, _start_s, 1e-8) < 0 ||
      Double::compare(s, _end_s, 1e-8) > 0) {
    AERROR << "Evalutuat s location " << s
           << ", is out of trajectory frenet frame range (" << _start_s << ", "
           << _end_s << ")";
    return false;
  }

  // linear bound interpolation
  uint32_t lower_index = FindIndex(s);
  const double s_low = _evaluated_knots[lower_index];
  const double s_high = _evaluated_knots[lower_index + 1];
  double weight = (Double::compare(s_high, s_low, 1e-8) <= 0
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
      std::lower_bound(_evaluated_knots.begin() + 1, _evaluated_knots.end(), s);
  return std::min(
             static_cast<uint32_t>(_evaluated_knots.size() - 1),
             static_cast<uint32_t>(upper_bound - _evaluated_knots.begin())) -
         1;
}

void QpFrenetFrame::Clear() {
  _reference_line = nullptr;
  _speed_profile = nullptr;
  _decision_data = nullptr;
  _evaluated_knots.clear();
  _discretized_veh_loc.clear();
  _hdmap_bound.clear();
  _static_obstacle_bound.clear();
  _dynamic_obstacle_bound.clear();
}

}  // namespace planning
}  // namespace apollo
