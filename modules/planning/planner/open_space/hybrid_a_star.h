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
#include <memory>

#include "modules/common/log.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

struct OpenSpaceConf {
  // for Hybrid A Star Warm Start
  double xy_grid_resolution;
  double phi_grid_resolution;
  double obstacle_grid_resolution;
  double max_x;
  double max_y;
  double max_phi;
  double min_x;
  double min_y;
  double min_phi;
}
class HybridAStar {
 public:
  explicit HybridAStar(Node3d start_node, Node3d end_node,
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
  std::unique_ptr<Node3d> start_node_;
  std::unique_ptr<Node3d> end_node_;
  auto cmp = [](SmartPtr<Node3d> left, SmartPtr<Node3d> right) {
    return left->GetCost() <= right->GetCost();
  };
  std::priority_queue<SmartPtr<Node3d>, std::vector<SmartPtr<Node3d>>, decltype(cmp)> open_pq_(cmp);
  std::map<double, SmartPtr<Node3d>> open_set_;
  std::map<double, SmartPtr<Node3d>> close_set_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNER_OPEN_SPACE_HYBRID_A_STAR_H_