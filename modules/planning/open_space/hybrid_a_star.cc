/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/*
 * hybrid_a_star.cc
 */

/*
 * Inspired by
 * 
 * Dolgov, Dmitri, et al. “Path Planning for Autonomous Vehicles in Unknown
 * Semi-Structured Environments.” The International Journal of Robotics
 * Research, vol. 29, no. 5, 2010, pp. 485–501., doi:10.1177/0278364909359210.
 * 
 * Xiaojing, et al. "Optimization-Based Collision Avoidance" and its implementation.
 */

#include "modules/planning/open_space/hybrid_a_star.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;

HybridAStar::HybridAStar() {
  CHECK(common::util::GetProtoFromFile(FLAGS_planner_open_space_config_filename,
                                       &open_space_conf_))
      << "Failed to load open space planner config file "
      << FLAGS_planner_open_space_config_filename;
  reed_shepp_generator_.reset(new ReedShepp(vehicle_param_, open_space_conf_));
  next_node_num_ = open_space_conf_.warm_start_config().next_node_num();
  max_steer_ = open_space_conf_.warm_start_config().max_steering();
  step_size_ = open_space_conf_.warm_start_config().step_size();
  xy_grid_resolution_ =
      open_space_conf_.warm_start_config().xy_grid_resolution();
  back_penalty_ = open_space_conf_.warm_start_config().back_penalty();
  gear_switch_penalty_ =
      open_space_conf_.warm_start_config().gear_switch_penalty();
  steer_penalty_ = open_space_conf_.warm_start_config().steer_penalty();
  steer_change_penalty_ =
      open_space_conf_.warm_start_config().steer_change_penalty();
}

bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node,
                                    ReedSheppPath* reeds_shepp_to_end) {
  if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                          reeds_shepp_to_end)) {
    AINFO << "ShortestRSP failed";
    return false;
  }
  if (!RSPCheck(reeds_shepp_to_end)) {
    return false;
  }
  return true;
}

bool HybridAStar::ReedSheppHeuristic(std::shared_ptr<Node3d> current_node,
                                     ReedSheppPath* reeds_shepp_to_end) {
  if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                          reeds_shepp_to_end)) {
    AINFO << "ShortestRSP failed";
    return false;
  }
  return true;
}

bool HybridAStar::RSPCheck(const ReedSheppPath* reeds_shepp_to_end) {
  for (std::size_t i = 0; i < reeds_shepp_to_end->x.size(); i++) {
    std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(
        new Node3d(reeds_shepp_to_end->x[i], reeds_shepp_to_end->y[i],
                   reeds_shepp_to_end->phi[i], open_space_conf_));
    if (!ValidityCheck(node)) {
      return false;
    }
  }
  return true;
}

bool HybridAStar::ValidityCheck(std::shared_ptr<Node3d> node) {
  for (const auto& obstacle_box : (*obstacles_).Items()) {
    if (node->GetBoundingBox(vehicle_param_)
            .HasOverlap(obstacle_box->PerceptionBoundingBox())) {
      return false;
    }
  }
  return true;
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
    const ReedSheppPath* reeds_shepp_to_end,
    std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x.back(), reeds_shepp_to_end->y.back(),
      reeds_shepp_to_end->phi.back(), reeds_shepp_to_end->x,
      reeds_shepp_to_end->y, reeds_shepp_to_end->phi, open_space_conf_));
  end_node->SetPre(current_node);
  end_node->SetTrajCost(CalculateRSPCost(reeds_shepp_to_end));
  close_set_.insert(std::make_pair(end_node->GetIndex(), end_node));
  return end_node;
}

std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
    std::shared_ptr<Node3d> current_node, std::size_t next_node_index) {
  double steering = 0.0;
  double index = 0.0;
  double traveled_distance = 0.0;
  if (next_node_index < next_node_num_ / 2) {
    steering = -max_steer_ +
               (2 * max_steer_ / (next_node_num_ / 2 - 1)) * next_node_index;
    traveled_distance = 1 * step_size_;
  } else {
    index = next_node_index - next_node_num_ / 2;
    steering =
        -max_steer_ + (2 * max_steer_ / (next_node_num_ / 2 - 1)) * index;
    traveled_distance = -1 * step_size_;
  }
  // take above motion primitive to generate a curve driving the car to a
  // different grid
  double arc = std::sqrt(2) * xy_grid_resolution_;
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;
  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();
  intermediate_x.emplace_back(last_x);
  intermediate_y.emplace_back(last_y);
  intermediate_phi.emplace_back(last_phi);
  for (std::size_t i = 0; i < arc / step_size_; i++) {
    double next_x = last_x + traveled_distance * std::cos(last_phi);
    double next_y = last_y + traveled_distance * std::sin(last_phi);
    double next_phi = common::math::NormalizeAngle(
        last_phi +
        traveled_distance / vehicle_param_.wheel_base() * std::tan(steering));
    intermediate_x.emplace_back(next_x);
    intermediate_y.emplace_back(next_y);
    intermediate_phi.emplace_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }
  std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
      new Node3d(last_x, last_y, last_phi, intermediate_x, intermediate_y,
                 intermediate_phi, open_space_conf_));
  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0);
  next_node->SetSteer(steering);
  return next_node;
}

void HybridAStar::CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                                    std::shared_ptr<Node3d> next_node,
                                    const ReedSheppPath* reeds_shepp_to_end) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  if (next_node->GetDirec()) {
    piecewise_cost += xy_grid_resolution_;
  } else {
    piecewise_cost += xy_grid_resolution_ * back_penalty_;
  }
  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost += gear_switch_penalty_;
  }
  piecewise_cost += steer_penalty_ * std::abs(next_node->GetSteer());
  piecewise_cost += steer_change_penalty_ *
                    std::abs(next_node->GetSteer() - current_node->GetSteer());
  next_node->SetTrajCost(current_node->GetTrajCost() + piecewise_cost);
  // evaluate heuristic cost
  next_node->SetHeuCost(NonHoloNoObstacleHeuristic(reeds_shepp_to_end));
}

double HybridAStar::NonHoloNoObstacleHeuristic(
    const ReedSheppPath* reeds_shepp_to_end) {
  return CalculateRSPCost(reeds_shepp_to_end);
}

double HybridAStar::CalculateRSPCost(const ReedSheppPath* reeds_shepp_to_end) {
  double RSP_cost = 0.0;
  for (std::size_t i = 0; i < reeds_shepp_to_end->segs_lengths.size(); i++) {
    if (reeds_shepp_to_end->segs_lengths[i] > 0.0) {
      RSP_cost += reeds_shepp_to_end->segs_lengths[i];
    } else {
      RSP_cost += reeds_shepp_to_end->segs_lengths[i] * back_penalty_;
    }
  }

  for (std::size_t i = 0; i < reeds_shepp_to_end->segs_lengths.size() - 1;
       i++) {
    if (reeds_shepp_to_end->segs_lengths[i] *
            reeds_shepp_to_end->segs_lengths[i + 1] <
        0.0) {
      RSP_cost += gear_switch_penalty_;
    }
  }
  // steering cost
  bool first_nonS_flag = false;
  char last_turning;
  for (std::size_t i = 0; i < reeds_shepp_to_end->segs_types.size(); i++) {
    if (reeds_shepp_to_end->segs_types[i] != 'S') {
      RSP_cost += steer_penalty_ * max_steer_;
      if (!first_nonS_flag) {
        last_turning = reeds_shepp_to_end->segs_types[i];
        first_nonS_flag = true;
        continue;
      }
      if (reeds_shepp_to_end->segs_types[i] != last_turning) {
        RSP_cost += 2 * steer_change_penalty_ * max_steer_;
      }
    }
  }
  return RSP_cost;
}
bool HybridAStar::GetResult(std::shared_ptr<Node3d> final_node,
                            Result* result) {
  std::shared_ptr<Node3d> current_node = final_node;
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  while (current_node->GetPreNode() != nullptr) {
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();
    if (x.size() == 0 || y.size() == 0 || phi.size() == 0) {
      AINFO << "result size check failed";
      return false;
    }
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    current_node = current_node->GetPreNode();
  }
  hybrid_a_x.push_back(current_node->GetX());
  hybrid_a_y.push_back(current_node->GetY());
  hybrid_a_phi.push_back(current_node->GetPhi());
  (*result).x = hybrid_a_x;
  (*result).y = hybrid_a_y;
  (*result).phi = hybrid_a_phi;
  return true;
}
bool HybridAStar::Plan(double sx, double sy, double sphi, double ex, double ey,
                       double ephi, ThreadSafeIndexedObstacles* obstacles,
                       Result* result) {
  // load nodes and obstacles
  std::vector<double> sx_vec{sx};
  std::vector<double> sy_vec{sy};
  std::vector<double> sphi_vec{sphi};
  std::vector<double> ex_vec{ex};
  std::vector<double> ey_vec{ey};
  std::vector<double> ephi_vec{ephi};
  start_node_.reset(
      new Node3d(sx, sy, sphi, sx_vec, sy_vec, sphi_vec, open_space_conf_));
  end_node_.reset(
      new Node3d(ex, ey, ephi, ex_vec, ey_vec, ephi_vec, open_space_conf_));
  obstacles_ = obstacles;
  if (!ValidityCheck(start_node_)) {
    AINFO << "start_node in collision with obstacles";
    return false;
  }
  if (!ValidityCheck(end_node_)) {
    AINFO << "end_node in collision with obstacles";
    return false;
  }
  // load open set and priority queue
  open_set_.insert(std::make_pair(start_node_->GetIndex(), start_node_));
  open_pq_.push(
      std::make_pair(start_node_->GetIndex(), start_node_->GetCost()));
  std::shared_ptr<Node3d> final_node;
  // Hybrid A* begins
  std::size_t explored_node_num = 0;
  double reeds_shepp_time = 0.0;
  double start_timestamp = 0.0;
  double end_timestamp = 0.0;
  while (!open_pq_.empty()) {
    // take out the lowest cost neighoring node
    std::size_t current_id = open_pq_.top().first;
    open_pq_.pop();
    std::shared_ptr<Node3d> current_node = open_set_[current_id];
    // check if a analystic curve could be connected from current configuration
    // to the end configuration without collision. if so, search ends.
    ReedSheppPath reeds_shepp_to_end;
    start_timestamp = Clock::NowInSeconds();
    if (AnalyticExpansion(current_node, &reeds_shepp_to_end)) {
      AINFO << "Reach the end configuration with Reed Sharp";
      // load the whole RSP as nodes and add to the close set
      final_node = LoadRSPinCS(&reeds_shepp_to_end, current_node);
      break;
    }
    close_set_.insert(std::make_pair(current_node->GetIndex(), current_node));
    end_timestamp = Clock::NowInSeconds();
    reeds_shepp_time += (end_timestamp - start_timestamp);
    for (std::size_t i = 0; i < next_node_num_; i++) {
      std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
      // boundary and validity check
      if (!ValidityCheck(next_node)) {
        continue;
      }
      // check if the node is already in the close set
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }

      if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
        explored_node_num++;
        start_timestamp = Clock::NowInSeconds();
        ReedSheppPath reeds_shepp_heuristic;
        ReedSheppHeuristic(next_node, &reeds_shepp_heuristic);
        CalculateNodeCost(current_node, next_node, &reeds_shepp_heuristic);
        end_timestamp = Clock::NowInSeconds();
        reeds_shepp_time += (end_timestamp - start_timestamp);
        open_set_.insert(std::make_pair(next_node->GetIndex(), next_node));
        open_pq_.push(
            std::make_pair(next_node->GetIndex(), next_node->GetCost()));
      } else {
        // reinitial the cost for rewiring
      }
    }
  }
  if (!GetResult(final_node, result)) {
    AINFO << "GetResult failed";
    return false;
  }
  AINFO << "explored node num is " << explored_node_num;
  AINFO << "reeds_shepp_time is " << reeds_shepp_time;
  return true;
}
}  // namespace planning
}  // namespace apollo
