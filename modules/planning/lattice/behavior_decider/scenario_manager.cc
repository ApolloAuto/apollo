/***************************************************************************
 *
 * Copyright (c) 2016 Baidu.com.0, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file scenario_manager.cpp
 **/

#include "modules/planning/lattice/behavior_decider/scenario_manager.h"
#include "modules/planning/lattice/behavior_decider/adc_master_scenario.h"

#include "modules/common/log.h"

namespace apollo {
namespace planning {

ScenarioManager::ScenarioManager() {}

void ScenarioManager::RegisterScenarios() {
  scenarios_.clear();
  scenarios_.resize(NUM_LEVELS);
  // level 0 features
  RegisterScenario<AdcMasterScenario>(LEVEL0);
}

void ScenarioManager::Reset() {
  scenarios_.clear();
  indexed_scenarios_.clear();
}

int ScenarioManager::ComputeWorldDecision(
    Frame* frame, const common::TrajectoryPoint& init_planning_point,
    const std::array<double, 3>& lon_init_state,
    const std::vector<common::PathPoint>& discretized_reference_line,
    std::vector<PlanningTarget>* const decisions) {
  RegisterScenarios();
  AINFO << "Register Scenarios Success";

  for (auto& level_scenario : scenarios_) {
    for (auto scenario : level_scenario) {
      scenario->Reset();

      if (!scenario->Init()) {
        AINFO << "scenario[" << scenario->Name() << "] init failed";
      } else {
        AINFO << "scenario[" << scenario->Name() << "] init success";
      }

      // check if exists
      if (!scenario->ScenarioExist()) {
        AINFO << "scenario[" << scenario->Name() << "] not exists";
      } else {
        AINFO << "scenario[" << scenario->Name() << "] does exists";
      }
      // compute decision
      if (0 ==
          scenario->ComputeScenarioDecision(
              frame, init_planning_point, lon_init_state,
              discretized_reference_line, decisions)) {
        AINFO << "scenario[" << scenario->Name()
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
