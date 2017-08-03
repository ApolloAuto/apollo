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
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SpeedPoint;
using apollo::common::Status;

DpStGraph::DpStGraph(const DpStSpeedConfig& dp_config)
    : dp_st_speed_config_(dp_config), dp_st_cost_(dp_config) {}

Status DpStGraph::Search(const StGraphData& st_graph_data,
                         DecisionData* const decision_data,
                         SpeedData* const speed_data, Obstacles* table) {
  init_point_ = st_graph_data.init_point();

  if (st_graph_data.path_data_length() <
      dp_st_speed_config_.total_path_length()) {
    dp_st_speed_config_.set_total_path_length(st_graph_data.path_data_length());
  }

  if (!InitCostTable().ok()) {
    const std::string msg = "Initialize cost table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  CalculatePointwiseCost(st_graph_data.st_graph_boundaries());

  if (!CalculateTotalCost(st_graph_data).ok()) {
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!retrieve_speed_profile(speed_data).ok()) {
    const std::string msg = "Retrieve best speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!get_object_decision(st_graph_data, *speed_data, table).ok()) {
    const std::string msg = "Get object decision by speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  return Status::OK();
}

Status DpStGraph::InitCostTable() {
  uint32_t dim_s = dp_st_speed_config_.matrix_dimension_s();
  uint32_t dim_t = dp_st_speed_config_.matrix_dimension_t();

  if (Double::compare(dp_st_speed_config_.total_path_length(), 0.0) == 0) {
    unit_s_ = 1e-8;
    uint32_t dim_s =
        std::min(dim_s, static_cast<uint32_t>(
                            dp_st_speed_config_.total_path_length() / unit_s_) +
                            1);
  } else {
    unit_s_ = dp_st_speed_config_.total_path_length() / dim_s;
  }

  unit_t_ = dp_st_speed_config_.total_time() /
            dp_st_speed_config_.matrix_dimension_t();
  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      dim_t, std::vector<StGraphPoint>(dim_s, StGraphPoint()));

  for (uint32_t i = 0; i < cost_table_.size(); ++i) {
    for (uint32_t j = 0; j < cost_table_[i].size(); ++j) {
      cost_table_[i][j].init(i, j, STPoint(unit_s_ * j, unit_t_ * i));
    }
  }

  return Status::OK();
}

void DpStGraph::CalculatePointwiseCost(
    const std::vector<StGraphBoundary>& boundaries) {
  // TODO(all): extract reference line from decision first
  std::vector<STPoint> reference_points;
  for (uint32_t i = 0; i < cost_table_.size(); ++i) {
    reference_points.emplace_back(unit_t_ * i * dp_st_speed_config_.max_speed(),
                                  unit_t_ * i);
  }

  for (uint32_t i = 0; i < cost_table_.size(); ++i) {
    for (uint32_t j = 0; j < cost_table_[i].size(); ++j) {
      double ref_cost = dp_st_cost_.GetReferenceCost(cost_table_[i][j].point(),
                                                     reference_points[i]);
      double obs_cost =
          dp_st_cost_.GetObstacleCost(cost_table_[i][j].point(), boundaries);
      cost_table_[i][j].set_reference_cost(ref_cost);
      cost_table_[i][j].set_obstacle_cost(obs_cost);
      cost_table_[i][j].set_total_cost(std::numeric_limits<double>::infinity());
    }
  }
}

Status DpStGraph::CalculateTotalCost(const StGraphData& st_graph_data) {
  // time corresponding to col, s corresponding to row
  for (uint32_t c = 0; c < cost_table_.size(); ++c) {
    for (uint32_t r = 0; r < cost_table_[c].size(); ++r) {
      CalculateCostAt(st_graph_data, c, r);
    }
  }
  return Status::OK();
}

void DpStGraph::CalculateCostAt(const StGraphData& st_graph_data,
                                const uint32_t c, const uint32_t r) {
  if (c == 0) {
    if (r == 0) {
      cost_table_[c][r].set_total_cost(0.0);
    } else {
      cost_table_[c][r].set_total_cost(std::numeric_limits<double>::infinity());
    }
    return;
  }

  // TODO(all): get speed limit from mapper
  double speed_limit =
      st_graph_data.speed_limit().get_speed_limit_by_s(unit_s_ * r);
  if (c == 1) {
    cost_table_[c][r].set_total_cost(
        cost_table_[c][r].obstacle_cost() + cost_table_[0][0].total_cost() +
        CalculateEdgeCostForSecondCol(r, speed_limit));
    cost_table_[c][r].set_pre_point(cost_table_[0][0]);
    return;
  }

  const uint32_t max_s_diff = static_cast<uint32_t>(
      dp_st_speed_config_.max_speed() * unit_t_ / unit_s_);
  const uint32_t r_low = (max_s_diff < r ? r - max_s_diff : 0);

  if (c == 2) {
    for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {
      double cost = cost_table_[c][r].obstacle_cost() +
                    cost_table_[c - 1][r_pre].total_cost() +
                    CalculateEdgeCostForThirdCol(r, r_pre, speed_limit);

      if (cost < cost_table_[c][r].total_cost()) {
        cost_table_[c][r].set_total_cost(cost);
        cost_table_[c][r].set_pre_point(cost_table_[c - 1][r_pre]);
      }
    }
    return;
  }

  for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {
    uint32_t lower_bound = 0;
    uint32_t upper_bound = 0;
    if (!CalculateFeasibleAccelRange(static_cast<double>(r_pre),
                                     static_cast<double>(r), &lower_bound,
                                     &upper_bound)) {
      continue;
    }

    for (uint32_t r_prepre = lower_bound; r_prepre <= upper_bound; ++r_prepre) {
      const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];
      if (!prepre_graph_point.pre_point()) {
        continue;
      } else {
        const STPoint& triple_pre_point =
            prepre_graph_point.pre_point()->point();
        const STPoint& prepre_point = cost_table_[c - 3][r_prepre].point();
        const STPoint& pre_point = cost_table_[c - 1][r_pre].point();
        const STPoint& curr_point = cost_table_[c][r].point();
        double cost = cost_table_[c][r].obstacle_cost() +
                      cost_table_[c - 1][r_pre].total_cost() +
                      CalculateEdgeCost(triple_pre_point, prepre_point,
                                        pre_point, curr_point, speed_limit);

        if (cost < cost_table_[c][r].total_cost()) {
          cost_table_[c][r].set_total_cost(cost);
          cost_table_[c][r].set_pre_point(cost_table_[c - 1][r_pre]);
        }
      }
    }
  }
}

bool DpStGraph::CalculateFeasibleAccelRange(const double r_pre,
                                            const double r_cur,
                                            uint32_t* const lower_bound,
                                            uint32_t* const upper_bound) const {
  double tcoef = unit_t_ * unit_t_ / unit_s_;
  double lval = std::max(
      2 * r_pre - r_cur + dp_st_speed_config_.max_deceleration() * tcoef, 0.0);
  double rval = std::min(
      2 * r_pre - r_cur + dp_st_speed_config_.max_acceleration() * tcoef,
      r_pre);

  if (rval < lval) {
    return false;
  }
  *lower_bound = static_cast<uint32_t>(lval);
  *upper_bound = static_cast<uint32_t>(rval);
  return true;
}

Status DpStGraph::retrieve_speed_profile(SpeedData* const speed_data) const {
  double min_cost = std::numeric_limits<double>::infinity();
  uint32_t n = cost_table_.back().size();
  uint32_t m = cost_table_.size();

  const StGraphPoint* best_end_point = nullptr;
  for (uint32_t i = 0; i < n; ++i) {
    const StGraphPoint& cur_point = cost_table_.back()[i];
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
    }
  }

  for (uint32_t i = 0; i < m; ++i) {
    const StGraphPoint& cur_point = cost_table_[i].back();
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
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
                                      const SpeedData& speed_profile,
                                      Obstacles* obstacles) const {
  if (speed_profile.speed_vector().size() < 2) {
    const std::string msg = "dp_st_graph failed to get speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const std::vector<StGraphBoundary>& obs_boundaries =
      st_graph_data.st_graph_boundaries();
  const std::vector<SpeedPoint>& speed_points = speed_profile.speed_vector();

  for (std::vector<StGraphBoundary>::const_iterator boundary_it =
           obs_boundaries.begin();
       boundary_it != obs_boundaries.end(); ++boundary_it) {
    CHECK_EQ(boundary_it->points().size(), 4);

    Obstacle* object_ptr = obstacles->Find(boundary_it->id());
    if (!object_ptr) {
      AERROR << "Failed to find object " << boundary_it->id();
      return Status(ErrorCode::PLANNING_ERROR,
                    "failed to find object from object_table");
    }
    if (boundary_it->points().front().x() <= 0) {
      ObjectDecisionType dec;
      dec.mutable_yield();
      object_ptr->MutableDecisions()->push_back(dec);
      continue;
    }
    double start_t = 0.0;
    double end_t = 0.0;
    boundary_it->GetBoundaryTimeScope(&start_t, &end_t);

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
      if (boundary_it->IsPointInBoundary(st_point)) {
        const std::string msg =
            "dp_st_graph failed: speed profile cross st_graph_boundaries.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }

      double s_upper = dp_st_speed_config_.total_path_length();
      double s_lower = 0.0;
      if (boundary_it->GetBoundarySRange(st_it->t(), &s_upper, &s_lower)) {
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

double DpStGraph::CalculateEdgeCost(const STPoint& first, const STPoint& second,
                                    const STPoint& third, const STPoint& forth,
                                    const double speed_limit) const {
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit) +
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

double DpStGraph::CalculateEdgeCostForSecondCol(
    const uint32_t row, const double speed_limit) const {
  double init_speed = init_point_.v();
  double init_acc = init_point_.a();
  const STPoint& pre_point = cost_table_[0][0].point();
  const STPoint& curr_point = cost_table_[1][row].point();
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit) +
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point,
                                             curr_point) +
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,
                                            curr_point);
}

double DpStGraph::CalculateEdgeCostForThirdCol(const uint32_t curr_row,
                                               const uint32_t pre_row,
                                               const double speed_limit) const {
  double init_speed = init_point_.v();
  const STPoint& first = cost_table_[0][0].point();
  const STPoint& second = cost_table_[1][pre_row].point();
  const STPoint& third = cost_table_[2][curr_row].point();
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit) +
         dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}

}  // namespace planning
}  // namespace apollo
