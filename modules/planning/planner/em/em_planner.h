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

#ifndef MODULES_PLANNING_PLANNER_EM_PLANNER_H_
#define MODULES_PLANNING_PLANNER_EM_PLANNER_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/common/proto/path_point.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/optimizer/optimizer.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_point.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class EMPlanner
 * @brief EMPlanner is an expectation maximization planner.
 */

class EMPlanner : public Planner {
 public:
  /**
   * @brief Constructor
   */
  EMPlanner();

  /**
   * @brief Destructor
   */
  virtual ~EMPlanner() = default;

  apollo::common::Status Init(const PlanningConfig& config) override;

  /**
   * @brief Overrode function Plan in parent class Planner.
   * @param start_point The trajectory point where planning starts
   * @param trajectory_pb The computed trajectory
   * @return OK if planning succeeds; error otherwise.
   */
  apollo::common::Status Plan(
      const apollo::common::TrajectoryPoint& start_point,
      ADCTrajectory* trajectory_pb) override;

 private:
  void RegisterOptimizers();

  std::vector<SpeedPoint> GenerateInitSpeedProfile(const double init_v,
                                                   const double init_a);

 private:
  apollo::common::util::Factory<OptimizerType, Optimizer> optimizer_factory_;
  std::vector<std::unique_ptr<Optimizer>> optimizers_;
};

}  // namespace planning
}  // namespace apollo

#endif  //* MODULES_PLANNING_PLANNER_EM_PLANNER_H_
