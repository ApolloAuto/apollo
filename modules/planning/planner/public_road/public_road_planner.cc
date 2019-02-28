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

#include "modules/planning/planner/public_road/public_road_planner.h"

#include <set>

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using common::Status;
using common::TrajectoryPoint;

Status PublicRoadPlanner::Init(const PlanningConfig& config) {
  config_ = config;
  std::set<ScenarioConfig::ScenarioType> supported_scenarios;
  const auto& public_road_config =
      config_.standard_planning_config().planner_public_road_config();

  for (int i = 0; i < public_road_config.scenario_type_size(); ++i) {
    const ScenarioConfig::ScenarioType scenario =
        public_road_config.scenario_type(i);
    supported_scenarios.insert(scenario);
  }
  scenario_manager_.Init(supported_scenarios);

  return Status::OK();
}

Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame,
                               ADCTrajectory* ptr_computed_trajectory) {
  DCHECK_NOTNULL(frame);
  scenario_manager_.Update(planning_start_point, *frame);
  scenario_ = scenario_manager_.mutable_scenario();
  auto result = scenario_->Process(planning_start_point, frame);

  if (FLAGS_enable_record_debug) {
    auto scenario_debug = ptr_computed_trajectory->mutable_debug()
                              ->mutable_planning_data()
                              ->mutable_scenario();
    scenario_debug->set_scenario_type(scenario_->scenario_type());
    scenario_debug->set_stage_type(scenario_->GetStage());
    scenario_debug->set_msg(scenario_->GetMsg());
  }

  if (result == scenario::Scenario::STATUS_DONE) {
    // only updates scenario manager when previous scenario's status is
    // STATUS_DONE
    scenario_manager_.Update(planning_start_point, *frame);
  } else if (result == scenario::Scenario::STATUS_UNKNOWN) {
    return Status(common::PLANNING_ERROR, "scenario returned unknown");
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
