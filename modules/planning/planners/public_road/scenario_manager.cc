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

#include "modules/planning/planners/public_road/scenario_manager.h"

#include <algorithm>
#include <string>
#include <vector>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/util/config_util.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

namespace apollo {
namespace planning {

using apollo::cyber::plugin_manager::PluginManager;

bool ScenarioManager::Init(const std::shared_ptr<DependencyInjector>& injector,
                           const PlannerPublicRoadConfig& planner_config) {
  if (init_) {
    return true;
  }
  injector_ = injector;
  for (int i = 0; i < planner_config.scenario_size(); i++) {
    auto scenario = PluginManager::Instance()->CreateInstance<Scenario>(
        ConfigUtil::GetFullPlanningClassName(
            planner_config.scenario(i).type()));
    ACHECK(scenario->Init(injector_, planner_config.scenario(i).name()))
        << "Can not init scenario" << planner_config.scenario(i).name();
    scenario_list_.push_back(scenario);
    if (planner_config.scenario(i).name() == "LANE_FOLLOW") {
      default_scenario_type_ = scenario;
    }
  }
  AINFO << "Load scenario list:" << planner_config.DebugString();
  current_scenario_ = default_scenario_type_;
  init_ = true;
  return true;
}

void ScenarioManager::Update(const common::TrajectoryPoint& ego_point,
                             Frame* frame) {
  CHECK_NOTNULL(frame);
  for (auto scenario : scenario_list_) {
    if (current_scenario_.get() == scenario.get() &&
        current_scenario_->GetStatus() ==
            ScenarioStatusType::STATUS_PROCESSING) {
      // The previous scenario has higher priority
      return;
    }
    if (scenario->IsTransferable(current_scenario_.get(), *frame)) {
      current_scenario_->Exit(frame);
      AINFO << "switch scenario from " << current_scenario_->Name() << " to "
            << scenario->Name();
      current_scenario_ = scenario;
      current_scenario_->Reset();
      current_scenario_->Enter(frame);
      return;
    }
  }
}

void ScenarioManager::Reset(Frame* frame) {
  if (current_scenario_) {
    current_scenario_->Exit(frame);
  }
  AINFO << "Reset to default scenario:" << default_scenario_type_->Name();
  default_scenario_type_->Reset();
  current_scenario_ = default_scenario_type_;
}
}  // namespace planning
}  // namespace apollo
