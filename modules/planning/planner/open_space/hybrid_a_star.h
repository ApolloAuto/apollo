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

#include "modules/planning/open_space/node3d.h"
#include "modules/planning/open_space/reeds_shepp_path.h"

#include "cybertron/common/log.h"
#include "cybertron/common/macros.h"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/constraint_checker/collision_checker.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planner_open_space_config.pb.h"

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
  bool Plan();

 private:
  // not complete
  bool AnalyticExpansion(std::shared_ptr<Node3d> current_node,
                         ReedSheppPath* reeds_shepp_to_end);
  // not complete
  std::vector<std::shared_ptr<Node3d>> KinemeticModelExpansion();
  // check collision and validity
  bool ValidityCheck(Node3d& node);
  // check Reeds Shepp path collision and validity
  bool RSPCheck(const ReedSheppPath* reeds_shepp_to_end);
  // load the whole RSP as nodes and add to the close set
  void LoadRSPinCS(const ReedSheppPath* reeds_shepp_to_end,
                   std::shared_ptr<Node3d> current_node);
  std::shared_ptr<Node3d> Next_node_generator(std::size_t next_node_index);
  double Cost();
  double HeuristicCost();
  double HoloObstacleHeuristic();
  double NonHoloNoObstacleHeuristic();
  double EuclidDist();
  Result GetResult();

 private:
  PlannerOpenSpaceConfig open_space_conf_;
  common::VehicleParam vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  std::size_t next_node_num_ = 0;
  std::vector<const Obstacle*> obstacles_;
  std::shared_ptr<Node3d> start_node_;
  std::shared_ptr<Node3d> end_node_;
  struct cmp {
    bool operator()(const std::pair<std::size_t, double> left,
                    const std::pair<std::size_t, double> right) const {
      return left.second <= right.second;
    }
  };
  std::priority_queue<std::pair<std::size_t, double>,
                      std::vector<std::pair<std::size_t, double>>, cmp>
      open_pq_;
  std::map<std::size_t, std::shared_ptr<Node3d>> open_set_;
  std::map<std::size_t, std::shared_ptr<Node3d>> close_set_;
  Result result_;
  std::unique_ptr<ReedShepp> reed_shepp_generator_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNER_OPEN_SPACE_HYBRID_A_STAR_H_