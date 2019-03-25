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

#include <string>
#include <vector>

#include "modules/planning/scenarios/traffic_light/unprotected_left_turn/stage_creep.h"

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/tasks/deciders/decider_creep.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

using common::TrajectoryPoint;
using common::time::Clock;
using hdmap::PathOverlap;
using perception::TrafficLight;

Stage::StageStatus TrafficLightUnprotectedLeftTurnStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    ADEBUG << "stage Creep finished because not enable.";
    return FinishStage();
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageCreep planning error";
  }

  /* TODO(all): to be added
  const auto& reference_line_info = frame->reference_line_info().front();

  // check if the traffic_light is still along reference_line
  std::string traffic_light_overlap_id =
      PlanningContext::GetScenarioInfo()->next_traffic_light_overlap.object_id;
  if (CheckTrafficLightDone(reference_line_info, traffic_light_overlap_id)) {
    return FinishScenario();
  }

  const double wait_time =
      Clock::NowInSeconds() - GetContext()->creep_start_time;
  const double timeout_sec = scenario_config_.creep_timeout_sec();
  auto* task = dynamic_cast<DeciderCreep*>(FindTask(TaskConfig::DECIDER_CREEP));
  if (task &&
      task->CheckCreepDone(
          *frame, reference_line_info,
          PlanningContext::GetScenarioInfo()->next_traffic_light_overlap.end_s,
          wait_time, timeout_sec)) {
    return FinishStage();
  }

  // set param for PROCEED_WITH_CAUTION_SPEED
  dynamic_cast<DeciderCreep*>(FindTask(TaskConfig::DECIDER_CREEP))
      ->SetProceedWithCautionSpeedParam(
          *frame, reference_line_info,
          PlanningContext::GetScenarioInfo()->next_traffic_light_overlap.end_s);
  */

  plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageCreep planning error";
  }
  return Stage::RUNNING;
}

Stage::StageStatus TrafficLightUnprotectedLeftTurnStageCreep::FinishStage() {
  next_stage_ =
      ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE;
  return Stage::FINISHED;
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
