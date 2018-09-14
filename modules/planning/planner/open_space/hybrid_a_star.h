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
 * hybrid_a_star.h
 */

#ifndef MODULES_PLANNING_PLANNER_OPEN_SPACE_HYBRID_A_STAR_H_
#define MODULES_PLANNING_PLANNER_OPEN_SPACE_HYBRID_A_STAR_H_

#include <map>
#include <queue>
#include <vector>

#include "modules/common/log.h"
#include "modules/common/math/box2d.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/collision_checker.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::math::Box2d;

class Node {
 public:
  explicit Node(double x, double y, double phi, double x_grid, double y_grid,
                double phi_grid);
  virtual ~Node() = default;
  Box2d GetBoundingBox(const common::VehicleParam& vehicle_param_);
  double GetCost() { return current_cost_ + heuristic_cost_; };
  std::size_t GetGridX() { return x_grid_; };
  std::size_t GetGridY() { return y_grid_; };
  std::size_t GetGridPhi() { return phi_grid_; };
  double GetX() { return x_; };
  double GetY() { return y_; };
  double GetPhi() { return phi_; };
  bool operator==(const Node* right) const;

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double phi_ = 0.0;
  std::size_t x_grid_ = 0;
  std::size_t y_grid_ = 0;
  std::size_t phi_grid_ = 0;
  double current_cost_ = 0.0;
  double heuristic_cost_ = 0.0;
  Node* pre_node = nullptr;
  double steering_ = 0.0;
  // true for moving forward and false for moving backward
  bool direction_ = true;
};

class HybridAStar {
 public:
  explicit HybridAStar(Node start_node, Node end_node,
                       std::vector<const Obstacle*> obstacles,
                       double xy_grid_resolution, double phi_grid_resolution,
                       double obstacle_grid_resolution);
  virtual ~HybridAStar() = default;
  Status Plan();

 private:
  AnalyticExpansion();
  KinemeticModelExpansion();
  double cost();
  double HeuristicCost();
  double HoloObstacleHeuristic();
  double NonHoloNoObstacleHeuristic();
  double EuclidDist();

 private:
  const common::VehicleParam& vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double xy_grid_resolution_;
  double phi_grid_resolution_;
  std::vector<const Obstacle*> obstacles_;
  std::unique_ptr<Node> start_node_;
  std::unique_ptr<Node> end_node_;
  auto cmp = [](Node* left, Node* right) {
    return left->GetCost() <= right->GetCost();
  };
  std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open_pq_(cmp);
  std::map<double, Node*> open_set_;
  std::map<double, Node*> close_set_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNER_OPEN_SPACE_HYBRID_A_STAR_H_