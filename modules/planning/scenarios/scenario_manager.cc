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

#include <limits>
#include <utility>
#include <vector>

#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"
#include "modules/planning/scenarios/side_pass/side_pass_scenario.h"
#include "modules/planning/scenarios/stop_sign_unprotected/stop_sign_unprotected_scenario.h"  // NOINT

namespace apollo {
namespace planning {
namespace scenario {

using apollo::hdmap::PathOverlap;

bool ScenarioManager::Init(
    const std::set<ScenarioConfig::ScenarioType>& supported_scenarios) {
  RegisterScenarios();
  default_scenario_type_ = ScenarioConfig::LANE_FOLLOW;
  supported_scenarios_ = supported_scenarios;
  current_scenario_ = CreateScenario(default_scenario_type_);
  return true;
}

std::unique_ptr<Scenario> ScenarioManager::CreateScenario(
    ScenarioConfig::ScenarioType scenario_type) {
  std::unique_ptr<Scenario> ptr;
  if (scenario_type == ScenarioConfig::LANE_FOLLOW) {
    ptr.reset(
        new LaneFollowScenario(config_map_[scenario_type], &scenario_context_));
  } else if (scenario_type == ScenarioConfig::SIDE_PASS) {
    ptr.reset(new scenario::side_pass::SidePassScenario(
        config_map_[scenario_type], &scenario_context_));
  } else if (scenario_type == ScenarioConfig::STOP_SIGN_UNPROTECTED) {
    ptr.reset(new scenario::stop_sign_protected::StopSignUnprotectedScenario(
        config_map_[scenario_type], &scenario_context_));
  } else {
    return nullptr;
  }
  if (ptr != nullptr) {
    ptr->Init();
  }
  return ptr;
}

void ScenarioManager::RegisterScenarios() {
  CHECK(Scenario::LoadConfig(FLAGS_scenario_lane_follow_config_file,
                             &config_map_[ScenarioConfig::LANE_FOLLOW]));
  CHECK(Scenario::LoadConfig(FLAGS_scenario_side_pass_config_file,
                             &config_map_[ScenarioConfig::SIDE_PASS]));
  CHECK(Scenario::LoadConfig(
      FLAGS_scenario_stop_sign_unprotected_config_file,
      &config_map_[ScenarioConfig::STOP_SIGN_UNPROTECTED]));
}

bool ScenarioManager::SelectChangeLaneScenario(
    const common::TrajectoryPoint& ego_point, const Frame& frame) {
  if (frame.reference_line_info().size() > 1) {
    if (current_scenario_->scenario_type() != ScenarioConfig::LANE_FOLLOW) {
      current_scenario_ = CreateScenario(ScenarioConfig::LANE_FOLLOW);
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
    auto scenario = CreateScenario(type);
    if (scenario->IsTransferable(*current_scenario_, ego_point, frame)) {
      current_scenario_ = std::move(scenario);
      return true;
    }
  }
  return false;
}

void ScenarioManager::Observe(const Frame& frame) {
  const auto& reference_line_info = frame.reference_line_info().front();

  // find next stop_sign_overlap
  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info.reference_line().map_path().stop_sign_overlaps();
  double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  double min_start_s = std::numeric_limits<double>::max();
  for (const PathOverlap& stop_sign_overlap : stop_sign_overlaps) {
    if (adc_front_edge_s - stop_sign_overlap.end_s <=
            conf_min_pass_s_distance_ &&
        stop_sign_overlap.start_s < min_start_s) {
      min_start_s = stop_sign_overlap.start_s;
      PlanningContext::GetScenarioInfo()->next_stop_sign_overlap =
          stop_sign_overlap;
    }
  }
}

void ScenarioManager::Update(const common::TrajectoryPoint& ego_point,
                             const Frame& frame) {
  CHECK(!frame.reference_line_info().empty());
  Observe(frame);
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
    current_scenario_ = CreateScenario(default_scenario_type_);
  }
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
