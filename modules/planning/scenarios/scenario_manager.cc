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
#include <string>
#include <utility>
#include <vector>

#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/bare_intersection/unprotected/bare_intersection_unprotected_scenario.h"
#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"
#include "modules/planning/scenarios/park/pull_over/pull_over_scenario.h"
#include "modules/planning/scenarios/park/valet_parking/valet_parking_scenario.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stop_sign_unprotected_scenario.h"
#include "modules/planning/scenarios/traffic_light/protected/traffic_light_protected_scenario.h"
#include "modules/planning/scenarios/traffic_light/unprotected_left_turn/traffic_light_unprotected_left_turn_scenario.h"
#include "modules/planning/scenarios/traffic_light/unprotected_right_turn/traffic_light_unprotected_right_turn_scenario.h"
#include "modules/planning/scenarios/util/util.h"

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

  switch (scenario_type) {
    case ScenarioConfig::LANE_FOLLOW:
      ptr.reset(new lane_follow::LaneFollowScenario(config_map_[scenario_type],
                                                    &scenario_context_));
      break;
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
      ptr.reset(
          new scenario::bare_intersection::BareIntersectionUnprotectedScenario(
              config_map_[scenario_type], &scenario_context_));
      break;
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
      ptr.reset(new scenario::stop_sign::StopSignUnprotectedScenario(
          config_map_[scenario_type], &scenario_context_));
      break;
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
      ptr.reset(new scenario::traffic_light::TrafficLightProtectedScenario(
          config_map_[scenario_type], &scenario_context_));
      break;
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
      ptr.reset(
          new scenario::traffic_light::TrafficLightUnprotectedLeftTurnScenario(
              config_map_[scenario_type], &scenario_context_));
      break;
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
      ptr.reset(
          new scenario::traffic_light::TrafficLightUnprotectedRightTurnScenario(
              config_map_[scenario_type], &scenario_context_));
      break;
    case ScenarioConfig::PULL_OVER:
      ptr.reset(new scenario::pull_over::PullOverScenario(
          config_map_[scenario_type], &scenario_context_));
      break;
    case ScenarioConfig::VALET_PARKING:
      ptr.reset(new scenario::valet_parking::ValetParkingScenario(
          config_map_[scenario_type], &scenario_context_));
      break;
    default:
      return nullptr;
  }

  if (ptr != nullptr) {
    ptr->Init();
  }
  return ptr;
}

void ScenarioManager::RegisterScenarios() {
  // lane_follow
  CHECK(Scenario::LoadConfig(FLAGS_scenario_lane_follow_config_file,
                             &config_map_[ScenarioConfig::LANE_FOLLOW]));

  // bare_intersection
  CHECK(Scenario::LoadConfig(
      FLAGS_scenario_bare_intersection_unprotected_config_file,
      &config_map_[ScenarioConfig::BARE_INTERSECTION_UNPROTECTED]));

  // stop_sign
  CHECK(Scenario::LoadConfig(
      FLAGS_scenario_stop_sign_unprotected_config_file,
      &config_map_[ScenarioConfig::STOP_SIGN_UNPROTECTED]));

  // traffic_light
  CHECK(Scenario::LoadConfig(
      FLAGS_scenario_traffic_light_protected_config_file,
      &config_map_[ScenarioConfig::TRAFFIC_LIGHT_PROTECTED]));
  CHECK(Scenario::LoadConfig(
      FLAGS_scenario_traffic_light_unprotected_left_turn_config_file,
      &config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN]));
  CHECK(Scenario::LoadConfig(
      FLAGS_scenario_traffic_light_unprotected_right_turn_config_file,
      &config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN]));

  // pull_over
  CHECK(Scenario::LoadConfig(FLAGS_scenario_pull_over_config_file,
                             &config_map_[ScenarioConfig::PULL_OVER]));

  // valet parking
  CHECK(Scenario::LoadConfig(FLAGS_scenario_valet_parking_config_file,
                             &config_map_[ScenarioConfig::VALET_PARKING]));
}

/*
 * @brief: function called by ScenarioSelfVote(),
 *         which selects scenario based on vote from each individual scenario.
 *         not in use now. but please do NOT delete the code yet
 */
/*
bool ScenarioManager::ReuseCurrentScenario(
   const common::TrajectoryPoint& ego_point, const Frame& frame) {
 return current_scenario_->IsTransferable(*current_scenario_, frame);
}
*/

/*
 * @brief: function called by ScenarioSelfVote(),
 *         which selects scenario based on vote from each individual scenario.
 *         not in use now. but please do NOT delete the code yet
 */
/*
bool ScenarioManager::SelectScenario(
    const ScenarioConfig::ScenarioType type,
    const common::TrajectoryPoint& ego_point,
    const Frame& frame) {
  if (current_scenario_->scenario_type() == type) {
    return true;
  }

  auto scenario = CreateScenario(type);
  if (scenario->IsTransferable(*current_scenario_, frame)) {
    AINFO << "switch to scenario: " << scenario->Name();
    current_scenario_ = std::move(scenario);
    return true;
  }
  return false;
}
*/

void ScenarioManager::Observe(const Frame& frame) {
  // init first_encountered_overlap_map_
  first_encountered_overlap_map_.clear();
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::PNC_JUNCTION ||
        overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::STOP_SIGN ||
        overlap.first == ReferenceLineInfo::YIELD_SIGN) {
      first_encountered_overlap_map_[overlap.first] = overlap.second;
    }
  }
}

void ScenarioManager::Update(const common::TrajectoryPoint& ego_point,
                             const Frame& frame) {
  CHECK(!frame.reference_line_info().empty());

  Observe(frame);

  ScenarioDispatch(ego_point, frame);
}

void ScenarioManager::ScenarioDispatch(const common::TrajectoryPoint& ego_point,
                                       const Frame& frame) {
  CHECK(!frame.reference_line_info().empty());

  // Update Scenario type.
  auto scenario_type = current_scenario_->UpdateScenarioType(
      ego_point, frame, config_map_, first_encountered_overlap_map_);

  // Check if it is supported by confs
  if (supported_scenarios_.find(scenario_type) == supported_scenarios_.end()) {
    scenario_type = default_scenario_type_;
  }

  ADEBUG << "select scenario: "
         << ScenarioConfig::ScenarioType_Name(scenario_type);

  // Update PlanningContext
  UpdatePlanningContext(frame, scenario_type);

  if (current_scenario_->scenario_type() != scenario_type) {
    current_scenario_ = CreateScenario(scenario_type);
  }
}

/*
 * @brief: select scenario based on vote from each individual scenario
 *         not in use now. but please do NOT delete the code yet
 */
/*
void ScenarioManager::ScenarioSelfVote(const common::TrajectoryPoint& ego_point,
                                       const Frame& frame) {
  CHECK(!frame.reference_line_info().empty());

  const auto& reference_line_info = frame.reference_line_info().front();

  // change lane case, currently default to LANE_FOLLOW in change lane case.
  // TODO(all) implement change lane scenario.
  // SelectChangeLaneScenario(reference_line_info);

  // non change lane case
  std::set<ScenarioConfig::ScenarioType> rejected_scenarios;
  if (current_scenario_->scenario_type() != default_scenario_type_ &&
      ReuseCurrentScenario(ego_point, frame)) {
    ADEBUG << "reuse current scenario: " << current_scenario_->Name();
    return;
  }
  rejected_scenarios.insert(current_scenario_->scenario_type());

  std::vector<ScenarioConfig::ScenarioType> preferred_scenarios;
  preferred_scenarios.push_back(ScenarioConfig::LANE_FOLLOW);

  const auto& first_overlaps = reference_line_info.FirstEncounteredOverlaps();
  for (const auto& overlap : first_overlaps) {
    // side_pass
    if (overlap.first == ReferenceLineInfo::OBSTACLE) {
      preferred_scenarios.push_back(ScenarioConfig::SIDE_PASS);
    }

    // stop_sign scenarios
    if (overlap.first == ReferenceLineInfo::STOP_SIGN) {
      preferred_scenarios.push_back(ScenarioConfig::STOP_SIGN_UNPROTECTED);
    }

    // traffic_light scenarios
    if (overlap.first == ReferenceLineInfo::SIGNAL) {
      preferred_scenarios.push_back(ScenarioConfig::TRAFFIC_LIGHT_PROTECTED);
      preferred_scenarios.push_back(
          ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN);
      preferred_scenarios.push_back(
          ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN);
    }
  }

  for (const auto& preferred_scenario : preferred_scenarios) {
    if (rejected_scenarios.find(preferred_scenario) !=
            rejected_scenarios.end() ||
        supported_scenarios_.count(preferred_scenario) == 0) {
      continue;
    }
    if (SelectScenario(preferred_scenario, ego_point, frame)) {
      AINFO << "select preferred scenario: "
            << ScenarioConfig::ScenarioType_Name(preferred_scenario);
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
    if (!FLAGS_enable_scenario_side_pass) {
      if (scenario == ScenarioConfig::SIDE_PASS) {
        continue;
      }
    }
    if (!FLAGS_enable_scenario_stop_sign) {
      if (scenario == ScenarioConfig::STOP_SIGN_UNPROTECTED) {
        continue;
      }
    }
    if (!FLAGS_enable_scenario_traffic_light) {
      if (scenario == ScenarioConfig::TRAFFIC_LIGHT_PROTECTED) {
        continue;
      }
      if (scenario == ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN) {
        continue;
      }
      if (scenario == ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN) {
        continue;
      }
    }

    if (SelectScenario(scenario, ego_point, frame)) {
      AINFO << "select transferable scenario: "
            << ScenarioConfig::ScenarioType_Name(scenario);
      return;
    } else {
      rejected_scenarios.insert(scenario);
    }
  }

  // finally use default transferrable scenario.
  if (current_scenario_->scenario_type() != default_scenario_type_) {
    AINFO << "select default scenario: "
          << ScenarioConfig::ScenarioType_Name(default_scenario_type_);
    current_scenario_ = CreateScenario(default_scenario_type_);
  }
}
*/

bool ScenarioManager::IsBareIntersectionScenario(
    const ScenarioConfig::ScenarioType& scenario_type) {
  return (scenario_type == ScenarioConfig::BARE_INTERSECTION_UNPROTECTED);
}

bool ScenarioManager::IsStopSignScenario(
    const ScenarioConfig::ScenarioType& scenario_type) {
  return (scenario_type == ScenarioConfig::STOP_SIGN_PROTECTED ||
          scenario_type == ScenarioConfig::STOP_SIGN_UNPROTECTED);
}

bool ScenarioManager::IsTrafficLightScenario(
    const ScenarioConfig::ScenarioType& scenario_type) {
  return (
      scenario_type == ScenarioConfig::TRAFFIC_LIGHT_PROTECTED ||
      scenario_type == ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN ||
      scenario_type == ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN);
}

void ScenarioManager::UpdatePlanningContext(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  // BareIntersection scenario
  UpdatePlanningContextBareIntersectionScenario(frame, scenario_type);

  // StopSign scenario
  UpdatePlanningContextStopSignScenario(frame, scenario_type);

  // TrafficLight scenario
  UpdatePlanningContextTrafficLightScenario(frame, scenario_type);
}

// update: bare_intersection status in PlanningContext
void ScenarioManager::UpdatePlanningContextBareIntersectionScenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  auto* bare_intersection = PlanningContext::Instance()
                                ->mutable_planning_status()
                                ->mutable_bare_intersection();

  if (!IsBareIntersectionScenario(scenario_type)) {
    bare_intersection->Clear();
    return;
  }

  if (scenario_type == current_scenario_->scenario_type()) {
    return;
  }

  // set to first_encountered pnc_junction
  const auto map_itr =
      first_encountered_overlap_map_.find(ReferenceLineInfo::PNC_JUNCTION);
  if (map_itr != first_encountered_overlap_map_.end()) {
    bare_intersection->set_current_pnc_junction_overlap_id(
        map_itr->second.object_id);
    ADEBUG << "Update PlanningContext with first_encountered pnc_junction["
           << map_itr->second.object_id << "] start_s["
           << map_itr->second.start_s << "]";
  }
}

// update: stop_sign status in PlanningContext
void ScenarioManager::UpdatePlanningContextStopSignScenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  if (!IsStopSignScenario(scenario_type)) {
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->Clear();
    return;
  }

  if (scenario_type == current_scenario_->scenario_type()) {
    return;
  }

  // set to first_encountered stop_sign
  const auto map_itr =
      first_encountered_overlap_map_.find(ReferenceLineInfo::STOP_SIGN);
  if (map_itr != first_encountered_overlap_map_.end()) {
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->set_current_stop_sign_overlap_id(map_itr->second.object_id);
    ADEBUG << "Update PlanningContext with first_encountered stop sign["
           << map_itr->second.object_id << "] start_s["
           << map_itr->second.start_s << "]";
  }
}

// update: traffic_light(s) status in PlanningContext
void ScenarioManager::UpdatePlanningContextTrafficLightScenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  if (!IsTrafficLightScenario(scenario_type)) {
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    return;
  }

  if (scenario_type == current_scenario_->scenario_type()) {
    return;
  }

  // get first_encountered traffic_light
  std::string current_traffic_light_overlap_id;
  const auto map_itr =
      first_encountered_overlap_map_.find(ReferenceLineInfo::SIGNAL);
  if (map_itr != first_encountered_overlap_map_.end()) {
    current_traffic_light_overlap_id = map_itr->second.object_id;
  }

  if (current_traffic_light_overlap_id.empty()) {
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    return;
  }

  // find all the traffic light at/within the same location/group
  const auto& reference_line_info = frame.reference_line_info().front();
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_itr = std::find_if(
      traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
      [&current_traffic_light_overlap_id](const hdmap::PathOverlap& overlap) {
        return overlap.object_id == current_traffic_light_overlap_id;
      });
  if (traffic_light_overlap_itr == traffic_light_overlaps.end()) {
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    return;
  }

  constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  const double current_traffic_light_overlap_start_s =
      traffic_light_overlap_itr->start_s;
  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    const double dist =
        traffic_light_overlap.start_s - current_traffic_light_overlap_start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->mutable_traffic_light()
          ->add_current_traffic_light_overlap_id(
              traffic_light_overlap.object_id);
      ADEBUG << "Update PlanningContext with first_encountered traffic_light["
             << traffic_light_overlap.object_id << "] start_s["
             << traffic_light_overlap.start_s << "]";
    }
  }
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
