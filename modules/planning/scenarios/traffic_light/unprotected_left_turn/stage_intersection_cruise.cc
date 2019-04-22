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

#include "modules/planning/scenarios/traffic_light/unprotected_left_turn/stage_intersection_cruise.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

Stage::StageStatus
TrafficLightUnprotectedLeftTurnStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageIntersectionCruise "
           << "plan error";
  }

  bool stage_done = stage_impl_.CheckDone(
      *frame, ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN, config_,
      true);
  if (stage_done) {
    return FinishStage();
  }
  return Stage::RUNNING;
}

Stage::StageStatus
TrafficLightUnprotectedLeftTurnStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
