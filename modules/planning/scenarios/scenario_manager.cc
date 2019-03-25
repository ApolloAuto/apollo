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

#include "modules/perception/proto/traffic_light_detection.pb.h"

#include "modules/common/time/time.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/scenarios/bare_intersection/unprotected/bare_intersection_unprotected_scenario.h"
#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"
#include "modules/planning/scenarios/side_pass/side_pass_scenario.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stop_sign_unprotected_scenario.h"
#include "modules/planning/scenarios/traffic_light/protected/traffic_light_protected_scenario.h"
#include "modules/planning/scenarios/traffic_light/unprotected_left_turn/traffic_light_unprotected_left_turn_scenario.h"
#include "modules/planning/scenarios/traffic_light/unprotected_right_turn/traffic_light_unprotected_right_turn_scenario.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/scenarios/valet_parking/valet_parking_scenario.h"

namespace apollo {
namespace planning {
namespace scenario {

using apollo::common::time::Clock;
using apollo::hdmap::PathOverlap;
using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;

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
    case ScenarioConfig::SIDE_PASS:
      ptr.reset(new scenario::side_pass::SidePassScenario(
          config_map_[scenario_type], &scenario_context_));
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

  // side_pass
  CHECK(Scenario::LoadConfig(FLAGS_scenario_side_pass_config_file,
                             &config_map_[ScenarioConfig::SIDE_PASS]));

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

  // valet parking
  CHECK(Scenario::LoadConfig(FLAGS_scenario_valet_parking_config_file,
                             &config_map_[ScenarioConfig::VALET_PARKING]));
}

ScenarioConfig::ScenarioType ScenarioManager::SelectChangeLaneScenario(
    const Frame& frame) {
  if (frame.reference_line_info().size() > 1) {
    // TODO(all): to be implemented
    return default_scenario_type_;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectStopSignScenario(
    const Frame& frame, const hdmap::PathOverlap& stop_sign_overlap) {
  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_stop_sign =
      stop_sign_overlap.start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_stop_sign[" << adc_distance_to_stop_sign
         << "] stop_sign[" << stop_sign_overlap.object_id
         << "] stop_sign_overlap_start_s[" << stop_sign_overlap.start_s << "]";

  const bool stop_sign_scenario =
      (adc_distance_to_stop_sign > 0 &&
       adc_distance_to_stop_sign <=
           config_map_[ScenarioConfig::STOP_SIGN_UNPROTECTED]
               .stop_sign_unprotected_config()
               .start_stop_sign_scenario_distance());
  const bool stop_sign_all_way = false;  // TODO(all)

  switch (current_scenario_->scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::CHANGE_LANE:
    case ScenarioConfig::SIDE_PASS:
      if (stop_sign_scenario) {
        return stop_sign_all_way ? ScenarioConfig::STOP_SIGN_PROTECTED
                                 : ScenarioConfig::STOP_SIGN_UNPROTECTED;
      }
      break;
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
      break;
    case ScenarioConfig::STOP_SIGN_PROTECTED:
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type();
      }
      break;
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioConfig::YIELD_SIGN_UNPROTECTED:
    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectTrafficLightScenario(
    const Frame& frame, const hdmap::PathOverlap& traffic_light_overlap) {
  auto scenario_config = config_map_[ScenarioConfig::TRAFFIC_LIGHT_PROTECTED]
                             .traffic_light_unprotected_right_turn_config();

  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const bool right_turn = (reference_line_info.GetPathTurnType(
                               adc_front_edge_s) == hdmap::Lane::RIGHT_TURN);
  const bool left_turn = (reference_line_info.GetPathTurnType(
                              adc_front_edge_s) == hdmap::Lane::LEFT_TURN);

  bool traffic_light_scenario = false;
  bool red_light = false;
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();

  // note: need iterate all lights to check no RED
  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    const double adc_distance_to_traffic_light =
        traffic_light_overlap.start_s - adc_front_edge_s;
    ADEBUG << "adc_distance_to_traffic_light[" << adc_distance_to_traffic_light
           << "] right_turn[" << right_turn << "] left_turn[" << left_turn
           << "]";

    // check distance
    if (adc_distance_to_traffic_light > 0 &&
        adc_distance_to_traffic_light <=
            config_map_[ScenarioConfig::TRAFFIC_LIGHT_PROTECTED]
                .traffic_light_protected_config()
                .start_traffic_light_scenario_distance()) {
      traffic_light_scenario = true;
      auto signal_color =
          scenario::GetSignal(traffic_light_overlap.object_id).color();
      ADEBUG << "traffic_light_id[" << traffic_light_overlap.object_id
             << "] start_s[" << traffic_light_overlap.start_s << "] color["
             << signal_color << "]";
      if (signal_color != TrafficLight::GREEN) {
        red_light = true;
        break;
      }
    }
  }

  switch (current_scenario_->scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::CHANGE_LANE:
    case ScenarioConfig::SIDE_PASS:
      if (traffic_light_scenario) {
        if (right_turn && red_light) {
          return ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN;
        }
        if (left_turn) {
          // TODO(all): switch when ready
          // return ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN;
          return ScenarioConfig::TRAFFIC_LIGHT_PROTECTED;
        }

        return ScenarioConfig::TRAFFIC_LIGHT_PROTECTED;
      }
      break;
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioConfig::STOP_SIGN_PROTECTED:
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
      break;
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type();
      }
      break;
    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectYieldSignScenario(
    const Frame& frame, const hdmap::PathOverlap& yield_sign_overlap) {
  // TODO(all)
  return current_scenario_->scenario_type();
}

ScenarioConfig::ScenarioType ScenarioManager::SelectBareIntersectionScenario(
    const Frame& frame, const hdmap::PathOverlap& pnc_junction_overlap) {
  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_pnc_junction =
      pnc_junction_overlap.start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_pnc_junction[" << adc_distance_to_pnc_junction
         << "] pnc_junction[" << pnc_junction_overlap.object_id
         << "] pnc_junction_overlap_start_s[" << pnc_junction_overlap.start_s
         << "]";

  // TODO(all)
  // const bool bare_junction_scenario =
  //    (adc_distance_to_pnc_junction > 0 && false);

  switch (current_scenario_->scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::CHANGE_LANE:
    case ScenarioConfig::SIDE_PASS:
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioConfig::STOP_SIGN_PROTECTED:
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectSidePassScenario(
    const Frame& frame) {
  // TODO(all): to be updated when SIDE_PASS obstacle decisions
  //            from ReferenceLine is ready
  if (current_scenario_->scenario_type() == ScenarioConfig::SIDE_PASS &&
      current_scenario_->IsTransferable(*current_scenario_, frame)) {
    return ScenarioConfig::SIDE_PASS;
  }

  auto scenario = CreateScenario(ScenarioConfig::SIDE_PASS);
  if (scenario->IsTransferable(*current_scenario_, frame)) {
    return ScenarioConfig::SIDE_PASS;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectValetParkingScenario(
    const Frame& frame) {
  // TODO(All) triger valet parking by route message definition as of now
  double parking_spot_range_to_start =
      config_map_[ScenarioConfig::VALET_PARKING]
          .valet_parking_config()
          .parking_spot_range_to_start();
  if (scenario::valet_parking::ValetParkingScenario::IsTransferable(
          frame, parking_spot_range_to_start)) {
    return ScenarioConfig::VALET_PARKING;
  }

  return default_scenario_type_;
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
  // read traffic light signal info
  ReadTrafficLight(frame);

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

void ScenarioManager::ReadTrafficLight(const Frame& frame) {
  PlanningContext::GetScenarioInfo()->traffic_lights.clear();
  const auto traffic_light_detection = frame.local_view().traffic_light;
  if (traffic_light_detection == nullptr) {
    ADEBUG << "traffic_light_detection is null";
    return;
  }

  const double delay =
      traffic_light_detection->header().timestamp_sec() - Clock::NowInSeconds();

  if (delay > signal_expire_time_sec_) {
    ADEBUG << "traffic signal is expired, delay[" << delay << "] seconds.";
    return;
  }

  for (int i = 0; i < traffic_light_detection->traffic_light_size(); i++) {
    const TrafficLight& traffic_light =
        traffic_light_detection->traffic_light(i);
    PlanningContext::GetScenarioInfo()->traffic_lights[traffic_light.id()] =
        &traffic_light;
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

  ////////////////////////////////////////
  // default: LANE_FOLLOW
  ScenarioConfig::ScenarioType scenario_type = default_scenario_type_;

  // check current_scenario (not switchable)
  switch (current_scenario_->scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::CHANGE_LANE:
      break;
    case ScenarioConfig::SIDE_PASS:
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioConfig::STOP_SIGN_PROTECTED:
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        scenario_type = current_scenario_->scenario_type();
      }
      break;
    default:
      break;
  }

  ////////////////////////////////////////
  // intersection scenarios
  if (scenario_type == default_scenario_type_) {
    hdmap::PathOverlap* traffic_sign_overlap = nullptr;
    hdmap::PathOverlap* pnc_junction_overlap = nullptr;
    ReferenceLineInfo::OverlapType overlap_type;

    const auto& reference_line_info = frame.reference_line_info().front();
    const auto& first_encountered_overlaps =
        reference_line_info.FirstEncounteredOverlaps();
    // note: first_encountered_overlaps already sorted
    for (const auto& overlap : first_encountered_overlaps) {
      if (overlap.first == ReferenceLineInfo::SIGNAL ||
          overlap.first == ReferenceLineInfo::STOP_SIGN ||
          overlap.first == ReferenceLineInfo::YIELD_SIGN) {
        overlap_type = overlap.first;
        traffic_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
        break;
      } else if (overlap.first == ReferenceLineInfo::PNC_JUNCTION) {
        pnc_junction_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
      }
    }

    if (traffic_sign_overlap) {
      switch (overlap_type) {
        case ReferenceLineInfo::STOP_SIGN:
          if (FLAGS_enable_scenario_stop_sign) {
            scenario_type =
                SelectStopSignScenario(frame, *traffic_sign_overlap);
          }
          break;
        case ReferenceLineInfo::SIGNAL:
          if (FLAGS_enable_scenario_traffic_light) {
            scenario_type =
                SelectTrafficLightScenario(frame, *traffic_sign_overlap);
          }
          break;
        case ReferenceLineInfo::YIELD_SIGN:
          // TODO(all): to be added
          // scenario_type = SelectYieldSignScenario(
          //     frame, *traffic_sign_overlap);
          break;
        default:
          break;
      }
    } else if (pnc_junction_overlap) {
      // bare intersection
      if (FLAGS_enable_scenario_bare_intersection) {
        if (reference_line_info.GetIntersectionRighoffRoad(
                *pnc_junction_overlap)) {
          scenario_type = default_scenario_type_;
        } else {
          scenario_type =
              SelectBareIntersectionScenario(frame, *pnc_junction_overlap);
        }
      }
    }
  }

  ////////////////////////////////////////
  // CHANGE_LANE scenario
  if (scenario_type == default_scenario_type_) {
    scenario_type = SelectChangeLaneScenario(frame);
  }

  ////////////////////////////////////////
  // SIDE_PASS scenario
  if (scenario_type == default_scenario_type_) {
    scenario_type = SelectSidePassScenario(frame);
  }

  ////////////////////////////////////////
  // VALET_PARKING scenario
  if (scenario_type == default_scenario_type_) {
    scenario_type = SelectValetParkingScenario(frame);
  }

  // Check if it is supported by confs
  if (supported_scenarios_.find(scenario_type) == supported_scenarios_.end()) {
    scenario_type = default_scenario_type_;
  }

  ADEBUG << "select scenario: "
         << ScenarioConfig::ScenarioType_Name(scenario_type);

  // update PlanningContext
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
  if (!IsStopSignScenario(scenario_type) &&
      !IsTrafficLightScenario(scenario_type)) {
    PlanningContext::GetScenarioInfo()->current_stop_sign_overlap =
        PathOverlap();
    PlanningContext::GetScenarioInfo()->current_traffic_light_overlaps.clear();
    PlanningContext::GetScenarioInfo()->stop_done_overlap_ids.clear();
    PlanningContext::GetScenarioInfo()->stop_sign_wait_for_obstacles.clear();
    return;
  }

  // StopSign scenario
  if (IsStopSignScenario(scenario_type)) {
    UpdatePlanningContextStopSignScenario(frame, scenario_type);
    return;
  }

  // TrafficLight scenario
  if (IsTrafficLightScenario(scenario_type)) {
    UpdatePlanningContextTrafficLightScenario(frame, scenario_type);
    return;
  }
}

// update: PlanningContext::GetScenarioInfo()->current_stop_sign_overlap
void ScenarioManager::UpdatePlanningContextStopSignScenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  const auto& reference_line_info = frame.reference_line_info().front();

  if (scenario_type != current_scenario_->scenario_type()) {
    PlanningContext::GetScenarioInfo()->current_stop_sign_overlap =
        PathOverlap();

    // set to first_encountered stop sign
    const auto map_itr =
        first_encountered_overlap_map_.find(ReferenceLineInfo::STOP_SIGN);
    if (map_itr != first_encountered_overlap_map_.end()) {
      PlanningContext::GetScenarioInfo()->current_stop_sign_overlap =
          map_itr->second;
      ADEBUG << "Update PlanningContext with first_encountered stop sign["
             << map_itr->second.object_id << "] start_s["
             << map_itr->second.start_s << "]";
    }
  } else {
    // refresh with current_stop_sign_overlap
    const std::string current_stop_sign_overlap_id =
        PlanningContext::GetScenarioInfo()->current_stop_sign_overlap.object_id;

    PlanningContext::GetScenarioInfo()->current_stop_sign_overlap =
        PathOverlap();
    const std::vector<PathOverlap>& stop_sign_overlaps =
        reference_line_info.reference_line().map_path().stop_sign_overlaps();
    auto stop_sign_overlap_itr = std::find_if(
        stop_sign_overlaps.begin(), stop_sign_overlaps.end(),
        [&current_stop_sign_overlap_id](const hdmap::PathOverlap& overlap) {
          return overlap.object_id == current_stop_sign_overlap_id;
        });
    if (stop_sign_overlap_itr != stop_sign_overlaps.end()) {
      PlanningContext::GetScenarioInfo()->current_stop_sign_overlap =
          *stop_sign_overlap_itr;
      ADEBUG << "refresh PlanningContext with current stop sign["
             << stop_sign_overlap_itr->object_id << "] start_s["
             << stop_sign_overlap_itr->start_s << "]";
    }
  }
}

// update: PlanningContext::GetScenarioInfo()->current_traffic_light_overlaps
void ScenarioManager::UpdatePlanningContextTrafficLightScenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  const auto& reference_line_info = frame.reference_line_info().front();

  std::string current_traffic_light_overlap_id;
  if (scenario_type != current_scenario_->scenario_type()) {
    const auto map_itr =
        first_encountered_overlap_map_.find(ReferenceLineInfo::SIGNAL);
    if (map_itr != first_encountered_overlap_map_.end()) {
      current_traffic_light_overlap_id = map_itr->second.object_id;
    }
  } else {
    // refresh with current_traffic_light_overlaps
    if (!PlanningContext::GetScenarioInfo()
             ->current_traffic_light_overlaps.empty()) {
      current_traffic_light_overlap_id = PlanningContext::GetScenarioInfo()
                                             ->current_traffic_light_overlaps[0]
                                             .object_id;
    }
  }

  if (current_traffic_light_overlap_id.empty()) {
    PlanningContext::GetScenarioInfo()->current_traffic_light_overlaps.clear();
    return;
  }

  // find all the traffic light at/within the same location/group
  // and refresh the overlap info
  PlanningContext::GetScenarioInfo()->current_traffic_light_overlaps.clear();

  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_itr = std::find_if(
      traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
      [&current_traffic_light_overlap_id](const hdmap::PathOverlap& overlap) {
        return overlap.object_id == current_traffic_light_overlap_id;
      });
  double current_traffic_light_overlap_start_s;
  if (traffic_light_overlap_itr != traffic_light_overlaps.end()) {
    current_traffic_light_overlap_start_s = traffic_light_overlap_itr->start_s;
  }

  constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    const double dist =
        traffic_light_overlap.start_s - current_traffic_light_overlap_start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      PlanningContext::GetScenarioInfo()
          ->current_traffic_light_overlaps.push_back(traffic_light_overlap);
      ADEBUG << "traffic_light[" << traffic_light_overlap.object_id
             << "] start_s[" << traffic_light_overlap.start_s << "]";
    }
  }
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
