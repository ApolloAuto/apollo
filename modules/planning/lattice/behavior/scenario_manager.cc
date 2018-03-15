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

#include "modules/planning/lattice/behavior/scenario_manager.h"

#include "modules/planning/lattice/behavior/ego_vehicle_scenario.h"
#include "modules/planning/lattice/behavior/signal_light_scenario.h"
#include "modules/common/log.h"

namespace apollo {
namespace planning {

ScenarioManager::ScenarioManager() {}

void ScenarioManager::RegisterScenarios() {
  scenarios_.clear();
  scenarios_.resize(NUM_LEVELS);
  // level 0 features
  RegisterScenario<EgoVehicleScenario>(LEVEL0);
  RegisterScenario<SignalLightScenario>(LEVEL0);
}

void ScenarioManager::Reset() {
  scenarios_.clear();
  indexed_scenarios_.clear();
}

int ScenarioManager::ComputeWorldDecision(
    Frame* frame, ReferenceLineInfo* const reference_line_info,
    PlanningTarget* planning_target) {
  RegisterScenarios();
  ADEBUG << "Register Scenarios Success";

  for (auto& level_scenario : scenarios_) {
    for (auto scenario : level_scenario) {
      scenario->Reset();

      if (!scenario->Init()) {
        AERROR << "scenario[" << scenario->Name() << "] init failed";
      } else {
        ADEBUG << "scenario[" << scenario->Name() << "] init success";
      }

      // check if exists
      if (!scenario->ScenarioExist()) {
        AERROR << "scenario[" << scenario->Name() << "] not exists";
      } else {
        ADEBUG << "scenario[" << scenario->Name() << "] does exists";
      }
      // compute decision
      if (0 ==
          scenario->ComputeScenarioDecision(frame, reference_line_info,
                                            planning_target)) {
        ADEBUG << "scenario[" << scenario->Name()
               << "] Success in computing decision";
      } else {
        AERROR << "scenario[" << scenario->Name()
               << "] Failed in computing decision";
      }
    }
  }
  return 0;
}
}  // namespace planning
}  // namespace apollo
