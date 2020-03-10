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

#include "modules/planning/scenarios/traffic_light/unprotected_left_turn/traffic_light_unprotected_left_turn_scenario.h"

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/scenarios/traffic_light/unprotected_left_turn/stage_approach.h"
#include "modules/planning/scenarios/traffic_light/unprotected_left_turn/stage_creep.h"
#include "modules/planning/scenarios/traffic_light/unprotected_left_turn/stage_intersection_cruise.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

using apollo::hdmap::HDMapUtil;

void TrafficLightUnprotectedLeftTurnScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();

  if (!GetScenarioConfig()) {
    AERROR << "fail to get scenario specific config";
    return;
  }

  const auto& traffic_light_status =
      PlanningContext::Instance()->planning_status().traffic_light();

  if (traffic_light_status.current_traffic_light_overlap_id().empty()) {
    AERROR << "Could not find traffic-light(s)";
    return;
  }

  context_.current_traffic_light_overlap_ids.clear();
  for (int i = 0;
       i < traffic_light_status.current_traffic_light_overlap_id_size(); i++) {
    const std::string traffic_light_overlap_id =
        traffic_light_status.current_traffic_light_overlap_id(i);
    hdmap::SignalInfoConstPtr traffic_light =
        HDMapUtil::BaseMap().GetSignalById(
            hdmap::MakeMapId(traffic_light_overlap_id));
    if (!traffic_light) {
      AERROR << "Could not find traffic light: " << traffic_light_overlap_id;
    }

    context_.current_traffic_light_overlap_ids.push_back(
        traffic_light_overlap_id);
  }

  init_ = true;
}

apollo::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
    TrafficLightUnprotectedLeftTurnScenario::s_stage_factory_;

void TrafficLightUnprotectedLeftTurnScenario::RegisterStages() {
  if (!s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_APPROACH,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new TrafficLightUnprotectedLeftTurnStageApproach(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new TrafficLightUnprotectedLeftTurnStageCreep(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new TrafficLightUnprotectedLeftTurnStageIntersectionCruise(
            config);
      });
}

std::unique_ptr<Stage> TrafficLightUnprotectedLeftTurnScenario::CreateStage(
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

/*
 * read scenario specific configs and set in context_ for stages to read
 */
bool TrafficLightUnprotectedLeftTurnScenario::GetScenarioConfig() {
  if (!config_.has_traffic_light_unprotected_left_turn_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(
      config_.traffic_light_unprotected_left_turn_config());
  return true;
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
