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
 * @file
 **/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Eigen"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/open_space/distance_approach_problem.h"
#include "modules/planning/open_space/hybrid_a_star.h"
#include "modules/planning/open_space/warm_start_problem.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/proto/planner_open_space_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

/*
Initially inspired by "Optimization-Based Collision Avoidance" from Xiaojing
Zhanga , Alexander Linigerb and Francesco Borrellia
*/

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

using apollo::common::Status;
/**
 * @class OpenSpacePlanner
 * @brief OpenSpacePlanner is a derived class of Planner.
 *        It reads a recorded trajectory from a trajectory file and
 *        outputs proper segment of the trajectory according to vehicle
 * position.
 */
class OpenSpacePlanner : public Planner {
 public:
  /**
   * @brief Constructor
   */
  OpenSpacePlanner() = default;

  /**
   * @brief Destructor
   */
  virtual ~OpenSpacePlanner() = default;

  std::string Name() override { return "OPEN_SPACE"; }

  apollo::common::Status Init(const PlanningConfig& config) override;

  /**
   * @brief override function Plan in parent class Planner.
   */
  apollo::common::Status Plan(
      const common::TrajectoryPoint& planning_init_point,
      Frame* frame) override;

 private:
  std::unique_ptr<::apollo::planning::HybridAStar> warm_start_;
  std::unique_ptr<::apollo::planning::DistanceApproachProblem>
      distance_approach_;
  common::VehicleState init_state_;
  const common::VehicleParam& vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
  apollo::planning::DistanceApproachConfig distance_approach_config_;
  double init_x_ = 0.0;
  double init_y_ = 0.0;
  double init_phi_ = 0.0;
  double init_v_ = 0.0;
  double init_steer_ = 0.0;
  double init_a_ = 0.0;
  size_t horizon_ = 0;
  double ts_ = 0;
  Eigen::MatrixXd ego_;
  Eigen::MatrixXd XYbounds_;
};

}  // namespace planning
}  // namespace apollo
