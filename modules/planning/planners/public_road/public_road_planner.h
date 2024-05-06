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

#pragma once

#include <memory>
#include <string>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/planning/planners/public_road/proto/planner_config.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/planning/planners/public_road/scenario_manager.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"
#include "modules/planning/planning_base/reference_line/reference_point.h"
#include "modules/planning/planning_interface_base/planner_base/planner.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

class Scenario;

/**
 * @class PublicRoadPlanner
 * @brief PublicRoadPlanner is an expectation maximization planner.
 */

class PublicRoadPlanner : public PlannerWithReferenceLine {
 public:
  /**
   * @brief Destructor
   */
  virtual ~PublicRoadPlanner() = default;

  void Stop() override {}

  std::string Name() override { return "PUBLIC_ROAD"; }

  common::Status Init(const std::shared_ptr<DependencyInjector>& injector,
                      const std::string& config_path = "") override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  common::Status Plan(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame,
                      ADCTrajectory* ptr_computed_trajectory) override;

  void Reset(Frame* frame) override { scenario_manager_.Reset(frame); }

 private:
  ScenarioManager scenario_manager_;
  PlannerPublicRoadConfig config_;
  Scenario* scenario_ = nullptr;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::PublicRoadPlanner,
                                     Planner)

}  // namespace planning
}  // namespace apollo
