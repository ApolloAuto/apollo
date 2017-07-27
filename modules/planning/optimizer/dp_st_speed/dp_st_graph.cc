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
 * @file dp_st_graph.cc
 **/

#include "modules/planning/optimizer/dp_st_speed/dp_st_graph.h"

#include <algorithm>
#include <limits>
#include <string>

#include "modules/common/log.h"
#include "modules/planning/common/data_center.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

DpStGraph::DpStGraph(const DpStSpeedConfig& dp_config)
    : _dp_st_speed_config(dp_config), _dp_st_cost(dp_config) {}

Status DpStGraph::search(const StGraphData& st_graph_data,
                         DecisionData* const decision_data,
                         SpeedData* const speed_data) {
  _init_point = st_graph_data.init_point();

  if (st_graph_data.path_data_length() <
      _dp_st_speed_config.total_path_length()) {
    _dp_st_speed_config.set_total_path_length(st_graph_data.path_data_length());
  }

  if (!init_cost_table().ok()) {
    const std::string msg = "Initialize cost table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!calculate_pointwise_cost(st_graph_data.obs_boundary()).ok()) {
    const std::string msg = "Calculate pointwise cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!calculate_total_cost().ok()) {
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!retrieve_speed_profile(speed_data).ok()) {
    const std::string msg = "Retrieve best speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!get_object_decision(st_graph_data, *speed_data).ok()) {
    const std::string msg = "Get object decision by speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  return Status::OK();
}

Status DpStGraph::init_cost_table() {
  std::uint32_t dim_s = _dp_st_speed_config.matrix_dimension_s();
  std::uint32_t dim_t = _dp_st_speed_config.matrix_dimension_t();

  if (Double::compare(_dp_st_speed_config.total_path_length(), 0.0) == 0) {
    _unit_s = 1e-8;
    std::uint32_t dim_s =
        std::min(dim_s, static_cast<std::uint32_t>(
                            _dp_st_speed_config.total_path_length() / _unit_s) +
                            1);
  } else {
    _unit_s = _dp_st_speed_config.total_path_length() / dim_s;
  }

  _unit_t = _dp_st_speed_config.total_time() /
            _dp_st_speed_config.matrix_dimension_t();
  _cost_table = std::vector<std::vector<StGraphPoint>>(
      dim_t, std::vector<StGraphPoint>(dim_s, StGraphPoint()));

  for (std::uint32_t i = 0; i < _cost_table.size(); ++i) {
    for (std::uint32_t j = 0; j < _cost_table[i].size(); ++j) {
      STPoint st_point;
      st_point.set_s(_unit_s * j);
      st_point.set_t(_unit_t * i);
      _cost_table[i][j].init(i, j, st_point);
    }
  }

  return Status::OK();
}

Status DpStGraph::calculate_pointwise_cost(
    const std::vector<StGraphBoundary>& boundaries) {
  // TODO(all): extract reference line from decision first
  std::vector<STPoint> reference_points;
  for (std::uint32_t i = 0; i < _cost_table.size(); ++i) {
    reference_points.emplace_back(_unit_t * i * _dp_st_speed_config.max_speed(),
                                  _unit_t * i);
  }

  for (std::uint32_t i = 0; i < _cost_table.size(); ++i) {
    for (std::uint32_t j = 0; j < _cost_table[i].size(); ++j) {
      double ref_cost = _dp_st_cost.reference_cost(_cost_table[i][j].point(),
                                                   reference_points[i]);
      double obs_cost =
          _dp_st_cost.obstacle_cost(_cost_table[i][j].point(), boundaries);
      _cost_table[i][j].set_reference_cost(ref_cost);
      _cost_table[i][j].set_obstacle_cost(obs_cost);
      _cost_table[i][j].set_total_cost(std::numeric_limits<double>::infinity());
    }
  }

  return Status::OK();
}

Status DpStGraph::calculate_total_cost() {
  // time corresponding to row, s corresponding to col
  for (std::uint32_t r = 0; r < _cost_table.size(); ++r) {
    for (std::uint32_t c = 0; c < _cost_table[r].size(); ++c) {
      calculate_total_cost(r, c);
    }
  }

  return Status::OK();
}

void DpStGraph::calculate_total_cost(const std::uint32_t r,
                                     const std::uint32_t c) {
  if (r == 0) {
    if (c == 0) {
      _cost_table[r][c].set_total_cost(0.0);
    } else {
      _cost_table[r][c].set_total_cost(std::numeric_limits<double>::infinity());
    }

    return;
  }
  // TODO(all): get speed limit from mapper @Liu Changchun
  double speed_limit = _dp_st_speed_config.max_speed();
  if (r == 1) {
    _cost_table[r][c].set_total_cost(
        _cost_table[r][c].obstacle_cost() + _cost_table[0][0].total_cost() +
        calculate_edge_cost_for_second_row(c, speed_limit));
    _cost_table[r][c].set_pre_point(_cost_table[0][0]);
    return;
  }

  std::uint32_t max_s_diff = static_cast<std::uint32_t>(
      _dp_st_speed_config.max_speed() * _unit_t / _unit_s);
  std::uint32_t c_low = (max_s_diff < c ? c - max_s_diff : 0);

  if (r == 2) {
    for (std::uint32_t c_pre = c_low; c_pre <= c; ++c_pre) {
      double cost = _cost_table[r][c].obstacle_cost() +
                    _cost_table[r - 1][c_pre].total_cost() +
                    calculate_edge_cost_for_third_row(c, c_pre, speed_limit);

      if (cost < _cost_table[r][c].total_cost()) {
        _cost_table[r][c].set_total_cost(cost);
        _cost_table[r][c].set_pre_point(_cost_table[r - 1][c_pre]);
      }
    }
    return;
  }

  for (std::uint32_t c_pre = c_low; c_pre <= c; ++c_pre) {
    std::uint32_t lower_bound = 0;
    std::uint32_t upper_bound = 0;
    if (!feasible_accel_range(static_cast<double>(c_pre),
                              static_cast<double>(c), &lower_bound,
                              &upper_bound)) {
      continue;
    }

    for (std::uint32_t c_prepre = lower_bound; c_prepre <= upper_bound;
         ++c_prepre) {
      const StGraphPoint& prepre_graph_point = _cost_table[r - 2][c_prepre];
      if (!prepre_graph_point.pre_point()) {
        continue;
      } else {
        const STPoint& triple_pre_point =
            prepre_graph_point.pre_point()->point();
        const STPoint& prepre_point = _cost_table[r - 3][c_prepre].point();
        const STPoint& pre_point = _cost_table[r - 1][c_pre].point();
        const STPoint& curr_point = _cost_table[r][c].point();
        double cost = _cost_table[r][c].obstacle_cost() +
                      _cost_table[r - 1][c_pre].total_cost() +
                      calculate_edge_cost(triple_pre_point, prepre_point,
                                          pre_point, curr_point, speed_limit);

        if (cost < _cost_table[r][c].total_cost()) {
          _cost_table[r][c].set_total_cost(cost);
          _cost_table[r][c].set_pre_point(_cost_table[r - 1][c_pre]);
        }
      }
    }
  }
}

bool DpStGraph::feasible_accel_range(const double c_pre, const double c_cur,
                                     std::uint32_t* const lower_bound,
                                     std::uint32_t* const upper_bound) const {
  double tcoef = _unit_t * _unit_t / _unit_s;
  double lval = std::max(
      2 * c_pre - c_cur + _dp_st_speed_config.max_deceleration() * tcoef, 0.0);
  double rval = std::min(
      2 * c_pre - c_cur + _dp_st_speed_config.max_acceleration() * tcoef,
      c_pre);

  if (rval < lval) {
    return false;
  }
  *lower_bound = static_cast<std::uint32_t>(lval);
  *upper_bound = static_cast<std::uint32_t>(rval);
  return true;
}

Status DpStGraph::retrieve_speed_profile(SpeedData* const speed_data) const {
  double min_cost = std::numeric_limits<double>::infinity();
  std::uint32_t n = _cost_table.back().size();
  const StGraphPoint* best_end_point = nullptr;
  for (std::uint32_t j = 0; j < n; ++j) {
    const StGraphPoint& cur_point = _cost_table.back()[j];
    if (cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
    }
  }

  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint* cur_point = best_end_point;
  while (cur_point != nullptr) {
    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_t(cur_point->point().t());
    speed_point.set_v(0);
    speed_point.set_a(0);
    speed_point.set_da(0);
    speed_profile.emplace_back(speed_point);

    cur_point = cur_point->pre_point();
  }
  std::reverse(speed_profile.begin(), speed_profile.end());

  if (Double::compare(speed_profile.front().t(), 0.0) != 0 ||
      Double::compare(speed_profile.front().s(), 0.0) != 0) {
    const std::string msg = "Fail to retrieve speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  speed_data->set_speed_vector(speed_profile);
  return Status::OK();
}

Status DpStGraph::get_object_decision(const StGraphData& st_graph_data,
                                      const SpeedData& speed_profile) const {
  if (speed_profile.speed_vector().size() < 2) {
    const std::string msg = "dp_st_graph failed to get speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const std::vector<StGraphBoundary>& obs_boundaries =
      st_graph_data.obs_boundary();
  const std::vector<SpeedPoint>& speed_points = speed_profile.speed_vector();
  for (std::vector<StGraphBoundary>::const_iterator obs_it =
           obs_boundaries.begin();
       obs_it != obs_boundaries.end(); ++obs_it) {
    CHECK_GE(obs_it->points().size(), 4);

    Obstacle* object_ptr =
        DataCenter::instance()->mutable_object_table()->get_obstacle(
            obs_it->id());
    if (!object_ptr) {
      AERROR << "Failed to find object " << obs_it->id();
      return Status(ErrorCode::PLANNING_ERROR,
                    "failed to find object from object_table");
    }
    if (obs_it->points().front().x() <= 0) {
      ObjectDecisionType dec;
      dec.mutable_yield();
      object_ptr->MutableDecisions()->push_back(dec);
      continue;
    }
    double start_t = 0.0;
    double end_t = 0.0;
    obs_it->get_boundary_time_scope(&start_t, &end_t);

    bool go_down = true;
    for (std::vector<SpeedPoint>::const_iterator st_it = speed_points.begin();
         st_it != speed_points.end(); ++st_it) {
      if (st_it->t() < start_t) {
        continue;
      }
      if (st_it->t() > end_t) {
        break;
      }

      STPoint st_point(st_it->s(), st_it->t());
      if (obs_it->IsPointInBoundary(st_point)) {
        const std::string msg =
            "dp_st_graph failed: speed profile cross obs_boundary.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }

      double s_upper = _dp_st_speed_config.total_path_length();
      double s_lower = 0.0;
      if (obs_it->get_boundary_s_range_by_time(st_it->t(), &s_upper,
                                               &s_lower)) {
        if (s_lower > st_it->s()) {
          go_down = true;
        } else if (s_upper < st_it->s()) {
          go_down = false;
        }
      }
    }
    if (go_down) {
      ObjectDecisionType dec;
      dec.mutable_yield();
      object_ptr->MutableDecisions()->push_back(dec);
    } else {
      ObjectDecisionType dec;
      dec.mutable_overtake();
      object_ptr->MutableDecisions()->push_back(dec);
    }
  }
  return Status::OK();
}

double DpStGraph::calculate_edge_cost(const STPoint& first,
                                      const STPoint& second,
                                      const STPoint& third,
                                      const STPoint& forth,
                                      const double speed_limit) const {
  return _dp_st_cost.speed_cost(third, forth, speed_limit) +
         _dp_st_cost.accel_cost_by_three_points(second, third, forth) +
         _dp_st_cost.jerk_cost_by_four_points(first, second, third, forth);
}

double DpStGraph::calculate_edge_cost_for_second_row(
    const uint32_t col, const double speed_limit) const {
  double init_speed = _init_point.v();
  double init_acc = _init_point.a();
  const STPoint& pre_point = _cost_table[0][0].point();
  const STPoint& curr_point = _cost_table[1][col].point();
  return _dp_st_cost.speed_cost(pre_point, curr_point, speed_limit) +
         _dp_st_cost.accel_cost_by_two_points(init_speed, pre_point,
                                              curr_point) +
         _dp_st_cost.jerk_cost_by_two_points(init_speed, init_acc, pre_point,
                                             curr_point);
}

double DpStGraph::calculate_edge_cost_for_third_row(
    const uint32_t curr_col, const uint32_t pre_col,
    const double speed_limit) const {
  double init_speed = _init_point.v();
  const STPoint& first = _cost_table[0][0].point();
  const STPoint& second = _cost_table[1][pre_col].point();
  const STPoint& third = _cost_table[2][curr_col].point();
  return _dp_st_cost.speed_cost(second, third, speed_limit) +
         _dp_st_cost.accel_cost_by_three_points(first, second, third) +
         _dp_st_cost.jerk_cost_by_three_points(init_speed, first, second,
                                               third);
}

}  // namespace planning
}  // namespace apollo
