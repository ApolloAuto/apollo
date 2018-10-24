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

/**
 * @file
 **/

#include <algorithm>
#include <limits>

#include "modules/planning/scenarios/stop_sign_unprotected/stop_sign_unprotected_scenario.h"  // NOINT

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/scenarios/stop_sign_unprotected/stop_sign_unprotected_stage.h"  // NOINT

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign_protected {

using common::TrajectoryPoint;
using common::time::Clock;
using hdmap::HDMapUtil;

using StopSignLaneVehicles =
    std::unordered_map<std::string, std::vector<std::string>>;

void StopSignUnprotectedScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();

  context_.next_stop_sign_overlap =
      PlanningContext::GetScenarioInfo()->next_stop_sign_overlap;
  const std::string stop_sign_overlap_id =
      context_.next_stop_sign_overlap.object_id;
  if (stop_sign_overlap_id.empty()) {
    return;
  }

  next_stop_sign_ = HDMapUtil::BaseMap().GetStopSignById(
      hdmap::MakeMapId(stop_sign_overlap_id));
  if (!next_stop_sign_) {
    AERROR << "Could not find stop sign: " << stop_sign_overlap_id;
    return;
  }

  GetAssociatedLanes(*next_stop_sign_);

  init_ = true;
}

apollo::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
    StopSignUnprotectedScenario::s_stage_factory_;

void StopSignUnprotectedScenario::RegisterStages() {
  if (!s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      ScenarioConfig::STOP_SIGN_UNPROTECTED_PRE_STOP,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StopSignUnprotectedPreStop(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::STOP_SIGN_UNPROTECTED_STOP,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StopSignUnprotectedStop(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::STOP_SIGN_UNPROTECTED_CREEP,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StopSignUnprotectedCreep(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StopSignUnprotectedIntersectionCruise(config);
      });
}

std::unique_ptr<Stage> StopSignUnprotectedScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config) {
  if (s_stage_factory_.Empty()) {
    RegisterStages();
  }
  auto ptr = s_stage_factory_.CreateObjectOrNull(stage_config.stage_type(),
                                                 stage_config);
  if (ptr) {
    ptr->SetContext(&context_);
  }
  return ptr;
}

bool StopSignUnprotectedScenario::IsTransferable(
    const Scenario& current_scenario,
    const common::TrajectoryPoint& ego_point,
    const Frame& frame) const {
  const std::string stop_sign_overlap_id =
      PlanningContext::GetScenarioInfo()->next_stop_sign_overlap.object_id;
  if (stop_sign_overlap_id.empty()) {
    return false;
  }

  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_stop_sign =
      context_.next_stop_sign_overlap.start_s - adc_front_edge_s;
  const double adc_speed =
      common::VehicleStateProvider::Instance()->linear_velocity();
  const uint32_t time_distance = static_cast<uint32_t>(ceil(
      adc_distance_to_stop_sign / adc_speed));

  switch (current_scenario.scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::CHANGE_LANE:
    case ScenarioConfig::SIDE_PASS:
    case ScenarioConfig::APPROACH:
      return (time_distance <= conf_start_stop_sign_timer_);
    case ScenarioConfig::STOP_SIGN_PROTECTED:
      return false;
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
      return true;
    case ScenarioConfig::TRAFFIC_LIGHT_LEFT_TURN_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_LEFT_TURN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_RIGHT_TURN_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_RIGHT_TURN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_GO_THROUGH:
    default:
      break;
  }

  return false;
}

}  // namespace stop_sign_protected
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
