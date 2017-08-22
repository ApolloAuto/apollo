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

#include "modules/planning/tasks/dp_st_speed/dp_st_graph.h"

#include <algorithm>
#include <limits>
#include <string>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

using ErrorCode = apollo::common::ErrorCode;
using SpeedPoint = apollo::common::SpeedPoint;
using Status = apollo::common::Status;
using VehicleParam = apollo::common::VehicleParam;
using Vec2d = apollo::common::math::Vec2d;

DpStGraph::DpStGraph(const DpStSpeedConfig& dp_config,
                     const StGraphData& st_graph_data,
                     const VehicleParam& vehicle_param,
                     const PathData& path_data)
    : dp_st_speed_config_(dp_config),
      st_graph_data_(st_graph_data),
      vehicle_param_(vehicle_param),
      path_data_(path_data),
      dp_st_cost_(dp_config),
      init_point_(st_graph_data.init_point()) {
  dp_st_speed_config_.set_total_path_length(
      std::fmin(dp_st_speed_config_.total_path_length(),
                st_graph_data_.path_data_length()));
}

Status DpStGraph::Search(PathDecision* const path_decision,
                         SpeedData* const speed_data) {
  if (!InitCostTable().ok()) {
    const std::string msg = "Initialize cost table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  CalculatePointwiseCost(st_graph_data_.st_graph_boundaries());

  if (!CalculateTotalCost().ok()) {
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!RetrieveSpeedProfile(speed_data).ok()) {
    const std::string msg = "Retrieve best speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!GetObjectDecision(*speed_data, path_decision).ok()) {
    const std::string msg = "Get object decision by speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  return Status::OK();
}

Status DpStGraph::InitCostTable() {
  uint32_t dim_s = dp_st_speed_config_.matrix_dimension_s();
  uint32_t dim_t = dp_st_speed_config_.matrix_dimension_t();

  if (Double::Compare(dp_st_speed_config_.total_path_length(), 0.0) == 0) {
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
  DCHECK_GT(dim_s, 2);
  DCHECK_GT(dim_t, 2);
  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      dim_t, std::vector<StGraphPoint>(dim_s, StGraphPoint()));

  double curr_t = 0.0;
  for (uint32_t i = 0; i < cost_table_.size(); ++i) {
    double curr_s = 0.0;
    for (uint32_t j = 0; j < cost_table_[i].size(); ++j) {
      cost_table_[i][j].Init(i, j, STPoint(curr_s, curr_t));
      curr_s += unit_s_;
    }
    curr_t += unit_t_;
  }

  return Status::OK();
}

void DpStGraph::CalculatePointwiseCost(
    const std::vector<StGraphBoundary>& boundaries) {
  // TODO(all): extract reference line from decision first
  std::vector<STPoint> reference_points;
  double curr_t = 0.0;
  for (uint32_t i = 0; i < cost_table_.size(); ++i) {
    reference_points.emplace_back(curr_t * dp_st_speed_config_.max_speed(),
                                  curr_t);
    curr_t += unit_t_;
  }

  for (uint32_t i = 0; i < cost_table_.size(); ++i) {
    for (uint32_t j = 0; j < cost_table_[i].size(); ++j) {
      double ref_cost = dp_st_cost_.GetReferenceCost(cost_table_[i][j].point(),
                                                     reference_points[i]);
      double obs_cost =
          dp_st_cost_.GetObstacleCost(cost_table_[i][j].point(), boundaries);
      cost_table_[i][j].SetReferenceCost(ref_cost);
      cost_table_[i][j].SetObstacleCost(obs_cost);
      cost_table_[i][j].SetTotalCost(std::numeric_limits<double>::infinity());
    }
  }
}

Status DpStGraph::CalculateTotalCost() {
  // s corresponding to row
  // time corresponding to col
  uint32_t c = 0;
  uint32_t next_highest_row = 0;
  uint32_t next_lowest_row = 0;

  while (c < cost_table_.size()) {
    uint32_t highest_row = 0;
    uint32_t lowest_row = cost_table_.back().size() - 1;
    for (uint32_t r = next_lowest_row; r <= next_highest_row; ++r) {
      CalculateCostAt(c, r);
      uint32_t h_r = 0;
      uint32_t l_r = 0;
      if (cost_table_.at(c).at(r).total_cost() <
          std::numeric_limits<double>::infinity()) {
        GetRowRange(r, c, &h_r, &l_r);
        highest_row = std::max(highest_row, h_r);
        lowest_row = std::min(lowest_row, l_r);
      }
    }
    ++c;
    next_highest_row = highest_row;
    next_lowest_row = std::max(next_lowest_row, lowest_row);
  }

  return Status::OK();
}

void DpStGraph::GetRowRange(const uint32_t curr_row, const uint32_t curr_col,
                            uint32_t* next_highest_row,
                            uint32_t* next_lowest_row) {
  const auto& curr_point = cost_table_[curr_col][curr_row];

  double v0 = 0.0;
  if (curr_col == 0) {
    v0 = init_point_.v();
  } else {
    const auto* pre_point = curr_point.pre_point();
    DCHECK_NOTNULL(pre_point);
    v0 = (curr_point.index_s() - pre_point->index_s()) * unit_s_ / unit_t_;
  }
  const double speed_coeff = 0.5 * unit_t_ * unit_t_;

  const double delta_s_upper_bound =
      v0 * unit_t_ + vehicle_param_.max_acceleration() * speed_coeff;
  *next_highest_row = curr_point.index_s() +
                      static_cast<uint32_t>(delta_s_upper_bound / unit_s_);
  if (*next_highest_row >= cost_table_.back().size()) {
    *next_highest_row = cost_table_.back().size() - 1;
  }

  const double delta_s_lower_bound = std::fmax(
      0.0, v0 * unit_t_ + vehicle_param_.max_deceleration() * speed_coeff);
  *next_lowest_row =
      *next_lowest_row + static_cast<int32_t>(delta_s_lower_bound / unit_s_);
  if (*next_lowest_row >= cost_table_.back().size()) {
    *next_lowest_row = cost_table_.back().size() - 1;
  }
}

void DpStGraph::CalculateCostAt(const uint32_t c, const uint32_t r) {
  if (c == 0) {
    DCHECK_EQ(r, 0) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_table_[c][r].SetTotalCost(0.0);
    return;
  }

  double speed_limit =
      st_graph_data_.speed_limit().GetSpeedLimitByS(unit_s_ * r);
  if (c == 1) {
    cost_table_[c][r].SetTotalCost(
        cost_table_[c][r].obstacle_cost() + cost_table_[0][0].total_cost() +
        CalculateEdgeCostForSecondCol(r, speed_limit));
    cost_table_[c][r].SetPrePoint(cost_table_[0][0]);
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
        cost_table_[c][r].SetTotalCost(cost);
        cost_table_[c][r].SetPrePoint(cost_table_[c - 1][r_pre]);
      }
    }
    return;
  }
  for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {
    if (std::isinf(cost_table_[c - 1][r_pre].total_cost()) ||
        cost_table_[c - 1][r_pre].pre_point() == nullptr) {
      continue;
    }

    const double curr_a = (cost_table_[c][r].index_s() +
                           cost_table_[c - 1][r_pre].pre_point()->index_s() -
                           2 * cost_table_[c - 1][r_pre].index_s()) *
                          unit_s_ / (unit_t_ * unit_t_);
    if (curr_a > vehicle_param_.max_acceleration() ||
        curr_a < vehicle_param_.max_deceleration()) {
      continue;
    }

    uint32_t lower_bound = 0;
    uint32_t upper_bound = 0;
    if (!CalculateFeasibleAccelRange(static_cast<double>(r_pre),
                                     static_cast<double>(r), &lower_bound,
                                     &upper_bound)) {
      continue;
    }

    for (uint32_t r_prepre = lower_bound; r_prepre <= upper_bound; ++r_prepre) {
      const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];
      if (std::isinf(prepre_graph_point.total_cost())) {
        continue;
      }

      if (!prepre_graph_point.pre_point()) {
        continue;
      }
      const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();
      const STPoint& prepre_point = prepre_graph_point.point();
      const STPoint& pre_point = cost_table_[c - 1][r_pre].point();
      const STPoint& curr_point = cost_table_[c][r].point();
      double cost = cost_table_[c][r].obstacle_cost() +
                    cost_table_[c - 1][r_pre].total_cost() +
                    CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                      curr_point, speed_limit);

      if (cost < cost_table_[c][r].total_cost()) {
        cost_table_[c][r].SetTotalCost(cost);
        cost_table_[c][r].SetPrePoint(cost_table_[c - 1][r_pre]);
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

Status DpStGraph::RetrieveSpeedProfile(SpeedData* const speed_data) const {
  double min_cost = std::numeric_limits<double>::infinity();
  uint32_t n = cost_table_.back().size();
  uint32_t m = cost_table_.size();

  const StGraphPoint* best_end_point = nullptr;
  for (uint32_t i = 0; i < n; ++i) {
    const StGraphPoint& cur_point = cost_table_.back()[i];
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  for (uint32_t i = 0; i < m; ++i) {
    const StGraphPoint& cur_point = cost_table_[i].back();
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
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
    speed_profile.emplace_back(speed_point);
    cur_point = cur_point->pre_point();
  }
  std::reverse(speed_profile.begin(), speed_profile.end());

  if (Double::Compare(speed_profile.front().t(), 0.0) != 0 ||
      Double::Compare(speed_profile.front().s(), 0.0) != 0) {
    const std::string msg = "Fail to retrieve speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  speed_data->set_speed_vector(speed_profile);
  return Status::OK();
}

Status DpStGraph::GetObjectDecision(const SpeedData& speed_profile,
                                    PathDecision* const path_decision) const {
  if (speed_profile.speed_vector().size() < 2) {
    const std::string msg = "dp_st_graph failed to get speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  for (const auto& boundary : st_graph_data_.st_graph_boundaries()) {
    CHECK_EQ(boundary.points().size(), 4);

    double start_t = boundary.min_t();
    double end_t = boundary.max_t();

    bool go_down = true;
    for (const auto& speed_point : speed_profile.speed_vector()) {
      if (speed_point.t() < start_t) {
        continue;
      }
      if (speed_point.t() > end_t) {
        break;
      }

      STPoint st_point(speed_point.s(), speed_point.t());
      if (boundary.IsPointInBoundary(st_point)) {
        const std::string msg =
            "dp_st_graph failed: speed profile cross st_graph_boundaries.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }

      double s_upper = dp_st_speed_config_.total_path_length();
      double s_lower = 0.0;
      if (boundary.GetBoundarySRange(speed_point.t(), &s_upper, &s_lower)) {
        if (s_lower > speed_point.s()) {
          go_down = true;
        } else if (s_upper < speed_point.s()) {
          go_down = false;
        }
      }
    }
    if (go_down) {
      if (CheckIsFollowByT(boundary)) {
        // FOLLOW decision
        const auto obstacle = path_decision->Find(boundary.id());
        const auto& obstacle_boundary = obstacle->perception_sl_boundary();
        ObjectDecisionType follow_decision;
        if (!CreateFollowDecision(*obstacle, obstacle_boundary,
                                  &follow_decision)) {
          AERROR << "Failed to create follow decision for boundary with id "
                 << boundary.id();
          return Status(ErrorCode::PLANNING_ERROR,
                        "faind to create follow decision");
        }
        if (!path_decision->AddLongitudinalDecision(
                "dp_st_graph", boundary.id(), follow_decision)) {
          AERROR << "Failed to add follow decision to object " << boundary.id();
          return Status(ErrorCode::PLANNING_ERROR,
                        "faind to add follow decision");
        }
      } else {
        // YIELD decision
        ObjectDecisionType yield_decision;
        if (!CreateYieldDecision(boundary, &yield_decision)) {
          AERROR << "Failed to create yield decision for boundary with id "
                 << boundary.id();
          return Status(ErrorCode::PLANNING_ERROR,
                        "faind to create yield decision");
        }
        if (!path_decision->AddLongitudinalDecision(
                "dp_st_graph", boundary.id(), yield_decision)) {
          AERROR << "Failed to add yield decision to object " << boundary.id();
          return Status(ErrorCode::PLANNING_ERROR,
                        "faind to add yield decision");
        }
      }
    } else {
      // OVERTAKE decision
      ObjectDecisionType overtake_decision;
      const auto obstacle = path_decision->Find(boundary.id());
      if (!CreateOvertakeDecision(*obstacle, boundary, &overtake_decision)) {
        AERROR << "Failed to create overtake decision for boundary with id "
               << boundary.id();
        return Status(ErrorCode::PLANNING_ERROR,
                      "faind to create overtake decision");
      }
      if (!path_decision->AddLongitudinalDecision("dp_st_graph", boundary.id(),
                                                  overtake_decision)) {
        AERROR << "Failed to add overtake decision to object " << boundary.id();
        return Status(ErrorCode::PLANNING_ERROR,
                      "faind to add overtake decision");
      }
    }
  }
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    if (!path_obstacle->HasLongitudinalDecision()) {
      ObjectDecisionType ignore_decision;
      ignore_decision.mutable_ignore();
      path_decision->AddLongitudinalDecision("dp_st_graph", path_obstacle->Id(),
                                             ignore_decision);
    }
  }
  return Status::OK();
}

bool DpStGraph::CreateFollowDecision(
    const PathObstacle& path_obstacle, const SLBoundary& obstacle_boundary,
    ObjectDecisionType* const follow_decision) const {
  DCHECK_NOTNULL(follow_decision);

  auto* follow = follow_decision->mutable_follow();

  // in seconds
  constexpr double kFollowTimeBuffer = 3.0;
  const auto& velocity = path_obstacle.Obstacle()->Perception().velocity();
  const double follow_speed =
      std::fmax(init_point_.v(), std::hypot(velocity.x(), velocity.y()));
  const double follow_distance_s =
      std::fmax(follow_speed * kFollowTimeBuffer, FLAGS_follow_min_distance) +
      vehicle_param_.front_edge_to_center();

  follow->set_distance_s(follow_distance_s);

  const double reference_line_fence_s =
      obstacle_boundary.start_s() - follow_distance_s;
  common::PathPoint path_point;
  if (!path_data_.GetPathPointWithRefS(std::fmax(0.0, reference_line_fence_s),
                                       &path_point)) {
    AERROR << "Failed to get path point from reference line s: "
           << reference_line_fence_s;
    return false;
  }
  auto* fence_point = follow->mutable_fence_point();
  fence_point->set_x(path_point.x());
  fence_point->set_y(path_point.y());
  fence_point->set_z(0.0);
  follow->set_fence_heading(path_point.theta());

  return true;
}

bool DpStGraph::CreateYieldDecision(
    const StGraphBoundary& boundary,
    ObjectDecisionType* const yield_decision) const {
  DCHECK_NOTNULL(yield_decision);

  auto* yield = yield_decision->mutable_yield();

  // in meters
  constexpr double kMinYieldDistance = 10.0;
  const double yield_distance_s = -1.0 * kMinYieldDistance;
  yield->set_distance_s(yield_distance_s);

  const double reference_line_fence_s = boundary.min_s() + yield_distance_s;
  common::PathPoint path_point;
  if (!path_data_.GetPathPointWithRefS(reference_line_fence_s, &path_point)) {
    AERROR << "Failed to get path point from reference line s "
           << reference_line_fence_s;
    return false;
  }

  yield->mutable_fence_point()->set_x(path_point.x());
  yield->mutable_fence_point()->set_y(path_point.y());
  yield->mutable_fence_point()->set_z(0.0);
  yield->set_fence_heading(path_point.theta());

  return true;
}

bool DpStGraph::CreateOvertakeDecision(
    const PathObstacle& path_obstacle, const StGraphBoundary& boundary,
    ObjectDecisionType* const overtake_decision) const {
  DCHECK_NOTNULL(overtake_decision);

  auto* overtake = overtake_decision->mutable_overtake();

  // in seconds
  constexpr double kOvertakeTimeBuffer = 3.0;
  // in meters
  constexpr double kMinOvertakeDistance = 10.0;

  const auto& velocity = path_obstacle.Obstacle()->Perception().velocity();
  const double obstacle_speed =
      Vec2d::CreateUnitVec2d(init_point_.path_point().theta())
          .InnerProd(Vec2d(velocity.x(), velocity.y()));

  const double overtake_distance_s = std::fmax(
      std::fmax(init_point_.v(), obstacle_speed) * kOvertakeTimeBuffer,
      kMinOvertakeDistance);
  overtake->set_distance_s(overtake_distance_s);

  const double reference_line_fence_s = boundary.max_s() + overtake_distance_s;
  common::PathPoint path_point;
  if (!path_data_.GetPathPointWithRefS(reference_line_fence_s, &path_point)) {
    AERROR << "Failed to get path point from reference line s "
           << reference_line_fence_s;
    return false;
  }

  overtake->mutable_fence_point()->set_x(path_point.x());
  overtake->mutable_fence_point()->set_y(path_point.y());
  overtake->mutable_fence_point()->set_z(0.0);
  overtake->set_fence_heading(path_point.theta());

  return true;
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

bool DpStGraph::CheckIsFollowByT(const StGraphBoundary& boundary) const {
  DCHECK_EQ(boundary.points().size(), 4);

  if (boundary.BottomLeftPoint().s() > boundary.BottomRightPoint().s()) {
    return false;
  }
  const double kFollowTimeEpsilon = 1e-3;
  if (boundary.min_t() > kFollowTimeEpsilon) {
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
