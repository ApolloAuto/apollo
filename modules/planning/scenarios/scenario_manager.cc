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

#include "modules/planning/scenarios/scenario_manager.h"

#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"

namespace apollo {
namespace planning {

bool ScenarioManager::Init() {
  RegisterScenarios();
  return true;
}

void ScenarioManager::RegisterScenarios() {
  scenario_factory_.Register(PlanningConfig::LANE_FOLLOW, []() -> Scenario* {
    return new LaneFollowScenario();
  });
}

void ScenarioManager::Update() {
  // TODO(Liangliang): update scenario here.
  scenario_ = scenario_factory_.CreateObject(PlanningConfig::LANE_FOLLOW);
}

PlanningConfig::ScenarioType ScenarioManager::DecideCurrentScenario() {
  return PlanningConfig::LANE_FOLLOW;
}

}  // namespace planning
}  // namespace apollo
