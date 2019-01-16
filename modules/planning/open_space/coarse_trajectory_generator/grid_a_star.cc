/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file
 */

#include "modules/planning/open_space/coarse_trajectory_generator/grid_a_star.h"

namespace apollo {
namespace planning {
GridAStar::GridAStar(const PlannerOpenSpaceConfig& open_space_conf) {
  xy_grid_resolution_ =
      open_space_conf.warm_start_config().grid_a_star_xy_resolution();
  node_radius_ = open_space_conf.warm_start_config().node_radius();
}

double GridAStar::EuclidHeuristic(const double& x, const double& y) {
  return std::sqrt((x - end_node_->GetGridX()) * (x - end_node_->GetGridX()) +
                   (y - end_node_->GetGridY()) * (y - end_node_->GetGridY()));
}

bool GridAStar::CheckConstraints(std::shared_ptr<Node2d> node) {
  if (obstacles_linesegments_vec_.size() == 0) {
    return true;
  }
  for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
    for (const common::math::LineSegment2d& linesegment :
         obstacle_linesegments) {
      if (linesegment.DistanceTo({node->GetGridX(), node->GetGridY()}) <
          node_radius_) {
        return false;
      }
    }
  }
  return true;
}

std::vector<std::shared_ptr<Node2d>> GridAStar::GenerateNextNodes(
    std::shared_ptr<Node2d> current_node) {
  double current_node_x = current_node->GetGridX();
  double current_node_y = current_node->GetGridY();
  double current_node_path_cost = current_node->GetPathCost();
  double diagonal_distance = std::sqrt(2.0);
  std::vector<std::shared_ptr<Node2d>> next_nodes;
  std::shared_ptr<Node2d> up =
      std::make_shared<Node2d>(current_node_x, current_node_y + 1.0, XYbounds_);
  up->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> up_right = std::make_shared<Node2d>(
      current_node_x + 1.0, current_node_y + 1.0, XYbounds_);
  up_right->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> right =
      std::make_shared<Node2d>(current_node_x + 1.0, current_node_y, XYbounds_);
  right->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> down_right = std::make_shared<Node2d>(
      current_node_x + 1.0, current_node_y - 1.0, XYbounds_);
  down_right->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> down =
      std::make_shared<Node2d>(current_node_x, current_node_y - 1.0, XYbounds_);
  down->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> down_left = std::make_shared<Node2d>(
      current_node_x - 1.0, current_node_y - 1.0, XYbounds_);
  down_left->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> left =
      std::make_shared<Node2d>(current_node_x - 1.0, current_node_y, XYbounds_);
  left->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> up_left = std::make_shared<Node2d>(
      current_node_x - 1.0, current_node_y + 1.0, XYbounds_);
  up_left->SetPathCost(current_node_path_cost + diagonal_distance);

  next_nodes.emplace_back(up);
  next_nodes.emplace_back(up_right);
  next_nodes.emplace_back(right);
  next_nodes.emplace_back(down_right);
  next_nodes.emplace_back(down);
  next_nodes.emplace_back(down_left);
  next_nodes.emplace_back(left);
  next_nodes.emplace_back(up_left);
  return next_nodes;
}

bool GridAStar::Plan(
    const double& sx, const double& sy, const double& ex, const double& ey,
    const std::vector<double>& XYbounds,
    const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
    double* optimal_path_cost) {
  // Clear and reload fields
  open_set_.clear();
  close_set_.clear();
  open_pq_ = decltype(open_pq_)();
  XYbounds_ = XYbounds;
  start_node_.reset(new Node2d(sx, sy, xy_grid_resolution_, XYbounds_));
  end_node_.reset(new Node2d(ex, ey, xy_grid_resolution_, XYbounds_));
  final_node_ = nullptr;
  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec;
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    size_t vertices_num = obstacle_vertices.size();
    std::vector<common::math::LineSegment2d> obstacle_linesegments;
    for (size_t i = 0; i < vertices_num - 1; ++i) {
      common::math::LineSegment2d line_segment = common::math::LineSegment2d(
          obstacle_vertices[i], obstacle_vertices[i + 1]);
      obstacle_linesegments.emplace_back(line_segment);
    }
    obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
  }
  obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);
  open_set_.insert(std::make_pair(start_node_->GetIndex(), start_node_));
  open_pq_.push(
      std::make_pair(start_node_->GetIndex(), start_node_->GetCost()));

  // Grid a star begins
  size_t explored_node_num = 0;
  while (!open_set_.empty()) {
    double current_id = open_pq_.top().first;
    open_pq_.pop();
    std::shared_ptr<Node2d> current_node = open_set_[current_id];
    // Check destination
    if (*(current_node) == *(end_node_)) {
      final_node_ = current_node;
      break;
    }
    close_set_.insert(std::make_pair(current_node->GetIndex(), current_node));
    std::vector<std::shared_ptr<Node2d>> next_nodes =
        std::move(GenerateNextNodes(current_node));
    for (auto& next_node : next_nodes) {
      if (!CheckConstraints(next_node)) {
        continue;
      }
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }
      if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
        ++explored_node_num;
        next_node->SetHeuristic(
            EuclidHeuristic(next_node->GetGridX(), next_node->GetGridY()));
        next_node->SetPreNode(current_node);
        open_set_.insert(std::make_pair(next_node->GetIndex(), next_node));
        open_pq_.push(
            std::make_pair(next_node->GetIndex(), next_node->GetCost()));
      }
    }
    if (final_node_ == nullptr) {
      AERROR << "Grid A searching return null ptr(open_set ran out)";
      return false;
    }
    *optimal_path_cost = final_node_->GetPathCost();
    ADEBUG << "explored node num is " << explored_node_num;
    return true;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo