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

#include <utility>

#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"
// #include "modules/planning/scenarios/side_pass/side_pass_scenario.h"
// #include
// "modules/planning/scenarios/stop_sign_unprotected/stop_sign_unprotected.h"

namespace apollo {
namespace planning {

bool ScenarioManager::Init(
    const std::set<ScenarioConfig::ScenarioType>& supported_scenarios) {
  RegisterScenarios();
  default_scenario_type_ = ScenarioConfig::LANE_FOLLOW;
  supported_scenarios_ = supported_scenarios;
  for (const auto scenario_type : supported_scenarios_) {
    CHECK(scenario_factory_.Contains(scenario_type));
  }
  current_scenario_ = scenario_factory_.CreateObject(default_scenario_type_);
  current_scenario_->Init();
  return true;
}

void ScenarioManager::RegisterScenarios() {
  scenario_factory_.Register(ScenarioConfig::LANE_FOLLOW, []() -> Scenario* {
    return new LaneFollowScenario();
  });
  // scenario_factory_.Register(ScenarioConfig::SIDE_PASS, []() -> Scenario* {
  //   return new SidePassScenario();
  // });
  // scenario_factory_.Register(
  //     ScenarioConfig::STOP_SIGN_UNPROTECTED,
  //     []() -> Scenario* { return new StopSignUnprotectedScenario(); });
}

bool ScenarioManager::SelectChangeLaneScenario(
    const common::TrajectoryPoint& ego_point, const Frame& frame) {
  if (frame.reference_line_info().size() > 1) {
    if (current_scenario_->scenario_type() != ScenarioConfig::LANE_FOLLOW) {
      current_scenario_ =
          scenario_factory_.CreateObject(ScenarioConfig::LANE_FOLLOW);
      current_scenario_->Init();
    }
    return true;
  } else {
    return false;
  }
}

bool ScenarioManager::ReuseCurrentScenario(
    const common::TrajectoryPoint& ego_point, const Frame& frame) {
  return current_scenario_->IsTransferable(*current_scenario_, ego_point,
                                           frame);
}

bool ScenarioManager::SelectScenario(const ScenarioConfig::ScenarioType type,
                                     const common::TrajectoryPoint& ego_point,
                                     const Frame& frame) {
  if (current_scenario_->scenario_type() == type) {
    return true;
  } else {
    auto scenario = scenario_factory_.CreateObject(type);
    if (scenario->IsTransferable(*current_scenario_, ego_point, frame)) {
      current_scenario_ = std::move(scenario);
      return true;
    }
  }
  return false;
}

void ScenarioManager::Update(const common::TrajectoryPoint& ego_point,
                             const Frame& frame) {
  CHECK(!frame.reference_line_info().empty());
  // change lane case, currently default to LANE_FOLLOW in change lane case.
  // TODO(all) implement change lane scenario.
  if (SelectChangeLaneScenario(ego_point, frame)) {
    ADEBUG << "Use change lane scenario (temporarily use LANE_FOLLOW)";
    return;
  }

  // non change lane case
  std::set<ScenarioConfig::ScenarioType> rejected_scenarios;
  if (current_scenario_->scenario_type() != default_scenario_type_ &&
      ReuseCurrentScenario(ego_point, frame)) {
    return;
  }
  rejected_scenarios.insert(current_scenario_->scenario_type());

  // prefer to use first encountered overlaps/objects
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_overlaps = reference_line_info.FirstEncounteredOverlaps();
  for (const auto& overlap : first_overlaps) {
    auto preferred_scenario = ScenarioConfig::LANE_FOLLOW;
    if (overlap.first == ReferenceLineInfo::STOP_SIGN) {
      preferred_scenario = ScenarioConfig::STOP_SIGN_UNPROTECTED;
    } else if (overlap.first == ReferenceLineInfo::OBSTACLE) {
      preferred_scenario = ScenarioConfig::SIDE_PASS;
    }
    if (rejected_scenarios.find(preferred_scenario) !=
            rejected_scenarios.end() ||
        supported_scenarios_.count(preferred_scenario) == 0) {
      continue;
    }
    if (SelectScenario(preferred_scenario, ego_point, frame)) {
      return;
    } else {
      rejected_scenarios.insert(preferred_scenario);
    }
  }

  // prefer to use first non-default transferrable scenario.
  for (const auto scenario : supported_scenarios_) {
    if (rejected_scenarios.find(scenario) != rejected_scenarios.end()) {
      continue;
    }
    if (SelectScenario(scenario, ego_point, frame)) {
      return;
    } else {
      rejected_scenarios.insert(scenario);
    }
  }

  // finally use default transferrable scenario.
  if (current_scenario_->scenario_type() != default_scenario_type_) {
    current_scenario_ = scenario_factory_.CreateObject(default_scenario_type_);
  }
}

}  // namespace planning
}  // namespace apollo
