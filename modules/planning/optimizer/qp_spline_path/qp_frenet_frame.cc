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
 * @file qp_frenet_frame.cpp
 **/
#include "optimizer/qp_spline_path_optimizer/qp_frenet_frame.h"
#include <algorithm>
#include <limits>
#include "common/planning_macros.h"
#include "math/double.h"

namespace apollo {
namespace planning {

ErrorCode QpFrenetFrame::init(const Environment& environment,
                              const ReferenceLine& reference_line,
                              const DecisionData& decision_data,
                              const SpeedData& speed_data,
                              const FrenetFramePoint& init_frenet_point,
                              const double start_s, const double end_s,
                              const double time_resolution,
                              const std::size_t num_points) {
  _environment = &environment;
  _speed_profile = &speed_data;
  _reference_line = &reference_line;
  _feasible_longitudinal_upper_bound =
      _reference_line->reference_map_line().length();
  _decision_data = &decision_data;
  _init_frenet_point = init_frenet_point;
  _start_s = start_s;
  _end_s = end_s;
  _time_resolution = time_resolution;
  QUIT_IF(num_points <= 1, ErrorCode::PLANNING_ERROR_FAILED, Level::ERROR,
          "Number of s points is too small to evaluate %ld", num_points);

  const double resolution = (_end_s - _start_s) / num_points;
  QUIT_IF(Double::compare(_start_s, _end_s, 1e-8) > 0,
          ErrorCode::PLANNING_ERROR_FAILED, Level::ERROR,
          "Not enough sl distance with start %f, end %f", _start_s, _end_s);

  QUIT_IF(!calcualate_discretized_veh_loc(), ErrorCode::PLANNING_ERROR_FAILED,
          Level::ERROR, "Fail to calculate discretized vehicle location!");

  const double inf = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i <= num_points; ++i) {
    double s = _start_s + i * resolution;
    _evaluated_knots.push_back(std::move(s));
    _hdmap_bound.push_back(std::make_pair(-inf, inf));
    _static_obstacle_bound.push_back(std::make_pair(-inf, inf));
    _dynamic_obstacle_bound.push_back(std::make_pair(-inf, inf));
  }

  // initialize calculation here
  QUIT_IF(!calculate_hd_map_bound(), ErrorCode::PLANNING_ERROR_FAILED,
          Level::ERROR, "Calculate hd map bound failed!");
  QUIT_IF(!calculate_static_obstacle_bound(), ErrorCode::PLANNING_ERROR_FAILED,
          Level::ERROR, "Calculate static obstacle bound failed!");
  QUIT_IF(!calculate_dynamic_obstacle_bound(), ErrorCode::PLANNING_ERROR_FAILED,
          Level::ERROR, "Calculate dynamic obstacle bound failed!");

  return ErrorCode::PLANNING_OK;
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
  return get_bound(s, _hdmap_bound, bound);
}

bool QpFrenetFrame::get_static_obstacle_bound(
    const double s, std::pair<double, double>* const bound) const {
  return get_bound(s, _static_obstacle_bound, bound);
}

bool QpFrenetFrame::get_dynamic_obstacle_bound(
    const double s, std::pair<double, double>* const bound) const {
  return get_bound(s, _dynamic_obstacle_bound, bound);
}

bool QpFrenetFrame::find_longitudinal_distance(const double time,
                                               SpeedPoint* const speed_point) {
  if (_speed_profile == nullptr) {
    return false;
  }

  if (!_speed_profile->get_speed_point_with_time(time, speed_point)) {
    return false;
  };

  return true;
}

bool QpFrenetFrame::calcualate_discretized_veh_loc() {
  double relative_time = 0.0;

  for (; relative_time < _speed_profile->total_time();
       relative_time += _time_resolution) {
    SpeedPoint veh_point;
    QUIT_IF(
        !_speed_profile->get_speed_point_with_time(relative_time, &veh_point),
        false, Level::ERROR, "Fail to get speed point at relative time %f",
        relative_time);
    veh_point.set_t(relative_time);
    _discretized_veh_loc.push_back(std::move(veh_point));
  }

  return true;
}

bool QpFrenetFrame::mapping_dynamic_obstacle_with_decision(
    const Obstacle& obstacle) {
  const std::vector<Decision>& decision = obstacle.decision();

  if (decision.size() != obstacle.prediction_trajectories().size()) {
    return false;
  }

  for (std::size_t i = 0; i < obstacle.prediction_trajectories().size(); ++i) {
    int nudge_side = 0;
    double buffer = decision[i].buffer();
    if (decision[i].decision_type() == Decision::DecisionType::GO_LEFT) {
      nudge_side = -1;
    } else if (decision[i].decision_type() ==
               Decision::DecisionType::GO_RIGHT) {
      nudge_side = 1;
    } else {
      continue;
    }
    const PredictionTrajectory& predicted_trajectory =
        obstacle.prediction_trajectories()[i];

    for (const SpeedPoint& veh_point : _discretized_veh_loc) {
      double time = veh_point.t();
      TrajectoryPoint trajectory_point = predicted_trajectory.evaluate(time);
      ::adu::common::math::Vec2d xy_point(trajectory_point.x(),
                                          trajectory_point.y());
      ::adu::common::math::Box2d obs_box(xy_point, trajectory_point.theta(),
                                         obstacle.length(), obstacle.width());
      // project obs_box on reference line
      std::vector<::adu::common::math::Vec2d> corners;
      obs_box.get_all_corners(&corners);
      std::vector<SLPoint> sl_corners;

      for (std::size_t i = 0; i < corners.size(); ++i) {
        SLPoint cur_point;
        Eigen::Vector2d xy_point(corners[i].x(), corners[i].y());
        QUIT_IF(
            !_reference_line->get_point_in_Frenet_frame(xy_point, &cur_point),
            false, Level::ERROR, "Fail to map xy point [%f, %f] to [%f, %f]",
            corners[i].x(), corners[i].y(), cur_point.s(), cur_point.l());
        // shift box base on buffer
        cur_point.set_l(cur_point.l() - buffer * nudge_side);
        sl_corners.push_back(std::move(cur_point));
      }

      for (std::size_t i = 0; i < sl_corners.size(); ++i) {
        SLPoint sl_first = sl_corners[i % sl_corners.size()];
        SLPoint sl_second = sl_corners[(i + 1) % sl_corners.size()];
        if (sl_first.s() < sl_second.s()) {
          std::swap(sl_first, sl_second);
        }

        std::pair<double, double> bound = map_lateral_constraint(
            sl_first, sl_second, nudge_side,
            veh_point.s() - _veh_config.back_edge_to_center(),
            veh_point.s() + _veh_config.front_edge_to_center());

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
        std::pair<std::size_t, std::size_t> update_index_range =
            find_interval(update_start_s, update_end_s);

        for (std::size_t j = update_index_range.first;
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

bool QpFrenetFrame::mapping_static_obstacle_with_decision(
    const Obstacle& obstacle) {
  const std::vector<Decision>& object_decisions = obstacle.decision();

  for (const Decision& decision : object_decisions) {
    if (decision.decision_type() == Decision::DecisionType::GO_LEFT) {
      ::adu::common::math::Box2d box = obstacle.bounding_box();
      std::vector<::adu::common::math::Vec2d> corners;
      box.get_all_corners(&corners);
      QUIT_IF(
          mapping_polygon(corners, decision.buffer(), -1,
                          &_static_obstacle_bound) != ErrorCode::PLANNING_OK,
          false, Level::ERROR,
          "fail to map polygon with id %d in qp frenet frame", obstacle.id());
    } else if (decision.decision_type() == Decision::DecisionType::GO_LEFT) {
      ::adu::common::math::Box2d box = obstacle.bounding_box();
      std::vector<::adu::common::math::Vec2d> corners;
      box.get_all_corners(&corners);
      QUIT_IF(
          mapping_polygon(corners, decision.buffer(), 1,
                          &_static_obstacle_bound) != ErrorCode::PLANNING_OK,
          false, Level::ERROR,
          "fail to map polygon with id %d in qp frenet frame", obstacle.id());
    }
  }

  return true;
}

ErrorCode QpFrenetFrame::mapping_polygon(
    const std::vector<::adu::common::math::Vec2d>& corners, const double buffer,
    const bool nudge_side,
    std::vector<std::pair<double, double>>* const bound_map) {
  std::vector<SLPoint> sl_corners;

  for (std::size_t i = 0; i < corners.size(); ++i) {
    SLPoint cur_point;
    Eigen::Vector2d xy_point(corners[i].x(), corners[i].y());
    QUIT_IF(_reference_line->get_point_in_Frenet_frame(xy_point, &cur_point),
            ErrorCode::PLANNING_ERROR_FAILED, Level::ERROR,
            "Fail to map xy point [%f, %f] to [%f, %f]", corners[i].x(),
            corners[i].y(), cur_point.s(), cur_point.l());
    // shift box base on buffer
    cur_point.set_l(cur_point.l() - buffer * nudge_side);
    sl_corners.push_back(std::move(cur_point));
  }

  for (std::size_t i = 0; i < sl_corners.size(); ++i) {
    ErrorCode ec = map_line(sl_corners[i % sl_corners.size()],
                            sl_corners[(i + 1) % sl_corners.size()], nudge_side,
                            bound_map);
    QUIT_IF(ec != ErrorCode::PLANNING_OK, ec, Level::ERROR,
            "Mapping box line (sl) (%f, %f), (%f, %f) failed",
            sl_corners[i].s(), sl_corners[i].l(), sl_corners[i + 1].s(),
            sl_corners[i + 1].l());
  }

  return ErrorCode::PLANNING_OK;
}

ErrorCode QpFrenetFrame::map_line(
    const SLPoint& start, const SLPoint& end, const int nudge_side,
    std::vector<std::pair<double, double>>* const constraint) {
  SLPoint near_point = (start.s() < end.s() ? start : end);
  SLPoint further_point = (start.s() < end.s() ? end : start);
  std::pair<std::size_t, std::size_t> impact_index =
      find_interval(near_point.s(), further_point.s());

  if ((near_point.s() < _start_s - _veh_config.back_edge_to_center() &&
       further_point.s() < _start_s - _veh_config.back_edge_to_center()) ||
      (near_point.s() > _end_s + _veh_config.front_edge_to_center() &&
       further_point.s() > _end_s + _veh_config.front_edge_to_center())) {
    return ErrorCode::PLANNING_OK;
  }

  double distance =
      std::max(std::abs(near_point.s() - further_point.s()), 1e-8);
  double weight = 0.0;

  for (std::size_t i = impact_index.first; i <= impact_index.second; ++i) {
    weight = std::abs((_evaluated_knots[i] - near_point.s())) / distance;
    weight = std::max(weight, 0.0);
    weight = std::min(weight, 1.0);
    double boundary =
        near_point.l() * (1 - weight) + further_point.l() * weight;

    if (nudge_side < 0) {
      boundary += _veh_config.width() / 2;
      (*constraint)[i].first = std::max(boundary, (*constraint)[i].first);
    } else if (nudge_side > 0) {
      boundary -= _veh_config.width() / 2;
      (*constraint)[i].second = std::min(boundary, (*constraint)[i].second);
    }

    if ((*constraint)[i].second < (*constraint)[i].first + 0.3) {
      if (i > 0) {
        _feasible_longitudinal_upper_bound =
            std::min(_evaluated_knots[i - 1] - kEpsilontol,
                     _feasible_longitudinal_upper_bound);
      } else {
        _feasible_longitudinal_upper_bound = _start_s;
        return ErrorCode::PLANNING_OK;
      }

      IDG_LOG(Level::ERROR,
              "current mapping constraint, sl point impact index \
                    %f, %f, %f, %f, %ld, %ld, %ld, constraint %f, %f",
              near_point.s(), near_point.l(), further_point.s(),
              further_point.l(), impact_index.first, impact_index.second, i,
              (*constraint)[i].first, (*constraint)[i].second);
    }
  }

  return ErrorCode::PLANNING_OK;
}

std::pair<double, double> QpFrenetFrame::map_lateral_constraint(
    const SLPoint& start, const SLPoint& end, const int nudge_side,
    const double s_start, const double s_end) {
  double inf = std::numeric_limits<double>::infinity();
  std::pair<double, double> result = std::make_pair(-inf, inf);

  if (start.s() > s_end || end.s() < s_start) {
    return result;
  } else {
    double s_front = std::max(start.s(), s_start);
    double s_back = std::min(end.s(), s_end);

    double weight_back = 0.0;
    double weight_front = 0.0;

    if (Double::compare((end.s() - start.s()), 0.0) > 0) {
      weight_back = (s_back - end.s()) / (end.s() - start.s());
      weight_front = (s_front - start.s()) / (end.s() - start.s());
    }

    SLPoint front = SLPoint::interpolate(start, end, weight_front);
    SLPoint back = SLPoint::interpolate(start, end, weight_back);

    if (nudge_side > 0) {
      double l = std::min(front.l(), back.l());
      result.second = l;
    } else if (nudge_side < 0) {
      double l = std::max(front.l(), back.l());
      result.first = l;
    }

    return result;
  }
}

std::pair<std::size_t, std::size_t> QpFrenetFrame::find_interval(
    const double start, const double end) const {
  double new_start = std::max(start - _veh_config.front_edge_to_center(),
                              _evaluated_knots.front());
  double new_end = std::min(end + _veh_config.back_edge_to_center(),
                            _evaluated_knots.back());
  std::size_t start_index = find_index(new_start);
  std::size_t end_index = find_index(new_end);

  if (end > _evaluated_knots[end_index] &&
      end_index + 1 != _evaluated_knots.size()) {
    end_index++;
  }

  return std::make_pair(start_index, end_index);
}

bool QpFrenetFrame::calculate_hd_map_bound() {
  for (std::size_t i = 0; i < _hdmap_bound.size(); ++i) {
    _hdmap_bound[i].first = -4.0;
    _hdmap_bound[i].second = 4.0;
  }
  return true;
}

bool QpFrenetFrame::calculate_static_obstacle_bound() {
  const std::vector<const Obstacle*> static_obs_list =
      _decision_data->static_obstacle();

  for (std::size_t i = 0; i < static_obs_list.size(); ++i) {
    const Obstacle* cur_obstacle = static_obs_list[i];
    QUIT_IF(mapping_static_obstacle_with_decision(*cur_obstacle), false,
            Level::ERROR,
            "mapping obstacle with id [%d] failed in qp frenet frame.",
            cur_obstacle->id());
  }

  return true;
}

bool QpFrenetFrame::calculate_dynamic_obstacle_bound() {
  const std::vector<const Obstacle*> dynamic_obs_list =
      _decision_data->dynamic_obstacle();

  for (std::size_t i = 0; i < dynamic_obs_list.size(); ++i) {
    const Obstacle* cur_obstacle = dynamic_obs_list[i];
    QUIT_IF(mapping_dynamic_obstacle_with_decision(*cur_obstacle), false,
            Level::ERROR,
            "mapping obstacle with id [%d] failed in qp frenet frame.",
            cur_obstacle->id());
  }

  return true;
}

bool QpFrenetFrame::get_bound(
    const double s, const std::vector<std::pair<double, double>>& bound_map,
    std::pair<double, double>* const bound) const {
  QUIT_IF(Double::compare(s, _start_s, 1e-8) < 0 ||
              Double::compare(s, _end_s, 1e-8) > 0,
          false, Level::ERROR,
          "Evalutuat s location %f, is out of trajectory frenet frame range "
          "(%f, %f)",
          s, _start_s, _end_s);

  // linear bound interpolation
  std::size_t lower_index = find_index(s);
  const double s_low = _evaluated_knots[lower_index];
  const double s_high = _evaluated_knots[lower_index + 1];
  double weight = (Double::compare(s_high, s_low, 1e-8) <= 0
                       ? 0
                       : (s - s_low) / (s_high - s_low));

  double low_first = bound_map[lower_index].first;
  double low_second = bound_map[lower_index].second;
  double high_first = bound_map[lower_index + 1].first;
  double high_second = bound_map[lower_index + 1].second;

  // If there is only one infinity in low and high point, then make it equal to
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

std::size_t QpFrenetFrame::find_index(const double s) const {
  auto upper_bound =
      std::lower_bound(_evaluated_knots.begin() + 1, _evaluated_knots.end(), s);
  return std::min(
             _evaluated_knots.size() - 1,
             static_cast<std::size_t>(upper_bound - _evaluated_knots.begin())) -
         1;
}

}  // namespace planning
}  // namespace apollo