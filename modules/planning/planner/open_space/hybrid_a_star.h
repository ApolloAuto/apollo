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
#include <memory>
#include <queue>
#include <vector>

#include "modules/common/log.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

struct Result {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
};

class HybridAStar {
 public:
  explicit HybridAStar(double sx, double sy, double sphi, double ex, double ey,
                       double ephi, std::vector<const Obstacle*> obstacles);
  virtual ~HybridAStar() = default;
  Status Plan();

 private:
  // not complete
  bool AnalyticExpansion(std::shared_ptr<Node3d> current_node);
  // not complete
  KinemeticModelExpansion();
  // check collision and validity
  bool Validitycheck(std::shared_ptr<Node3d> next_node);
  void Next_node_generator(std::size_t next_node_index);
  double Cost();
  double HeuristicCost();
  double HoloObstacleHeuristic();
  double NonHoloNoObstacleHeuristic();
  double EuclidDist();
  Result GetResult();

 private:
  PlannerOpenSpaceConfig open_space_conf_;
  const common::VehicleParam& vehicle_param_;
  std::size_t next_node_num_ = 0;
  std::vector<const Obstacle*> obstacles_;
  std::shared_ptr<Node3d> start_node_;
  std::shared_ptr<Node3d> end_node_;
  auto cmp = [](std::pair<std::size_t, double> left,
                std::pair<std::size_t, double> right) {
    return left.second <= right.second;
  };
  std::priority_queue<std::pair<std::size_t, double>,
                      std::vector<std::pair<std::size_t, double>>,
                      decltype(cmp)>
      open_pq_(cmp);
  std::map<std::size_t, shared_ptr<Node3d>> open_set_;
  std::map<std::size_t, shared_ptr<Node3d>> close_set_;
  Result result_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNER_OPEN_SPACE_HYBRID_A_STAR_H_