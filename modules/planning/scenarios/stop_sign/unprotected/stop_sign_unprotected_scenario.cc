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

#include "modules/planning/scenarios/stop_sign/unprotected/stop_sign_unprotected_scenario.h"

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "cyber/common/log.h"

#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stage_creep.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stage_intersection_cruise.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stage_pre_stop.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stage_stop.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign {

using common::TrajectoryPoint;
using hdmap::HDMapUtil;
using hdmap::StopSignInfo;
using hdmap::StopSignInfoConstPtr;

using StopSignLaneVehicles =
    std::unordered_map<std::string, std::vector<std::string>>;

apollo::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
    StopSignUnprotectedScenario::s_stage_factory_;

void StopSignUnprotectedScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();

  if (!GetScenarioConfig()) {
    AERROR << "fail to get scenario specific config";
    return;
  }

  const std::string stop_sign_overlap_id =
      PlanningContext::GetScenarioInfo()->next_stop_sign_overlap.object_id;
  if (stop_sign_overlap_id.empty()) {
    return;
  }

  context_.stop_sign_id = stop_sign_overlap_id;
  stop_sign_ = HDMapUtil::BaseMap().GetStopSignById(
      hdmap::MakeMapId(stop_sign_overlap_id));
  if (!stop_sign_) {
    AERROR << "Could not find stop sign: " << stop_sign_overlap_id;
    return;
  }
  context_.watch_vehicles.clear();

  GetAssociatedLanes(*stop_sign_);

  init_ = true;
}

void StopSignUnprotectedScenario::RegisterStages() {
  if (!s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      ScenarioConfig::STOP_SIGN_UNPROTECTED_PRE_STOP,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StagePreStop(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::STOP_SIGN_UNPROTECTED_STOP,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StageStop(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::STOP_SIGN_UNPROTECTED_CREEP,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StageCreep(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StageIntersectionCruise(config);
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
    const Scenario& current_scenario, const TrajectoryPoint& ego_point,
    const Frame& frame) {
  if (PlanningContext::GetScenarioInfo()
          ->next_stop_sign_overlap.object_id.empty()) {
    return false;
  }

  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double stop_sign_overlap_start_s =
      PlanningContext::GetScenarioInfo()->next_stop_sign_overlap.start_s;
  const double adc_distance_to_stop_sign =
      stop_sign_overlap_start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_stop_sign[" << adc_distance_to_stop_sign
         << "] stop_sign_overlap_start_s[" << stop_sign_overlap_start_s << "]";

  switch (current_scenario.scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::CHANGE_LANE:
    case ScenarioConfig::SIDE_PASS:
    case ScenarioConfig::APPROACH:
      return (adc_distance_to_stop_sign > 0 &&
              adc_distance_to_stop_sign <=
                  config_.stop_sign_unprotected_config()
                      .start_stop_sign_scenario_distance());
    case ScenarioConfig::STOP_SIGN_PROTECTED:
      return false;
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
      return (current_scenario.GetStatus() !=
              Scenario::ScenarioStatus::STATUS_DONE);
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    default:
      break;
  }
  return false;
}

/*
 * read scenario specific configs and set in context_ for stages to read
 */
bool StopSignUnprotectedScenario::GetScenarioConfig() {
  if (!config_.has_stop_sign_unprotected_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(config_.stop_sign_unprotected_config());
  return true;
}

/*
 * get all the lanes associated/guarded by a stop sign
 */
int StopSignUnprotectedScenario::GetAssociatedLanes(
    const StopSignInfo& stop_sign_info) {
  context_.associated_lanes.clear();

  std::vector<StopSignInfoConstPtr> associated_stop_signs;
  HDMapUtil::BaseMap().GetStopSignAssociatedStopSigns(stop_sign_info.id(),
                                                      &associated_stop_signs);

  for (const auto stop_sign : associated_stop_signs) {
    if (stop_sign == nullptr) {
      continue;
    }

    const auto& associated_lane_ids = stop_sign->OverlapLaneIds();
    for (const auto& lane_id : associated_lane_ids) {
      const auto lane = HDMapUtil::BaseMap().GetLaneById(lane_id);
      if (lane == nullptr) {
        continue;
      }
      for (const auto& stop_sign_overlap : lane->stop_signs()) {
        auto over_lap_info =
            stop_sign_overlap->GetObjectOverlapInfo(stop_sign.get()->id());
        if (over_lap_info != nullptr) {
          context_.associated_lanes.push_back(
              std::make_pair(lane, stop_sign_overlap));
          ADEBUG << "stop_sign: " << stop_sign_info.id().id()
                 << "; associated_lane: " << lane_id.id()
                 << "; associated_stop_sign: " << stop_sign.get()->id().id();
        }
      }
    }
  }

  return 0;
}

}  // namespace stop_sign
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
