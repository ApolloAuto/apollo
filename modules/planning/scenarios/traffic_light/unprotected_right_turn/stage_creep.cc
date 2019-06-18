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

#include "modules/planning/scenarios/traffic_light/unprotected_right_turn/stage_creep.h"

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/tasks/deciders/creep_decider/creep_decider.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

using common::TrajectoryPoint;
using common::time::Clock;
using hdmap::PathOverlap;
using perception::TrafficLight;

Stage::StageStatus TrafficLightUnprotectedRightTurnStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    return FinishStage();
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedRightTurnStageCreep planning error";
  }

  if (GetContext()->current_traffic_light_overlap_ids.size() == 0) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  PathOverlap* traffic_light = nullptr;
  bool traffic_light_all_green = true;
  for (const auto& traffic_light_overlap_id :
       GetContext()->current_traffic_light_overlap_ids) {
    // get overlap along reference line
    PathOverlap* current_traffic_light_overlap =
        scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                  traffic_light_overlap_id,
                                                  ReferenceLineInfo::SIGNAL);
    if (!current_traffic_light_overlap) {
      continue;
    }

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(
        current_traffic_light_overlap->start_s, false);

    auto signal_color = frame->GetSignal(traffic_light_overlap_id).color();
    ADEBUG << "traffic_light_overlap_id[" << traffic_light_overlap_id
           << "] start_s[" << current_traffic_light_overlap->start_s
           << "] color[" << signal_color << "]";

    // check on traffic light color
    if (signal_color != TrafficLight::GREEN) {
      traffic_light_all_green = false;
      break;
    }

    traffic_light = current_traffic_light_overlap;
  }

  if (traffic_light_all_green || traffic_light == nullptr) {
    return FinishStage();
  }

  // creep
  const double wait_time =
      Clock::NowInSeconds() - GetContext()->creep_start_time;
  const double timeout_sec = scenario_config_.creep_timeout_sec();
  auto* task = dynamic_cast<CreepDecider*>(FindTask(TaskConfig::CREEP_DECIDER));
  if (task &&
      task->CheckCreepDone(*frame, reference_line_info, traffic_light->end_s,
                           wait_time, timeout_sec)) {
    return FinishStage();
  }

  // set param for PROCEED_WITH_CAUTION_SPEED
  // dynamic_cast<CreepDecider*>(FindTask(TaskConfig::CREEP_DECIDER))
  //    ->SetProceedWithCautionSpeedParam(*frame, reference_line_info,
  //                                      traffic_light.end_s);

  plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedRightTurnStageCreep planning error";
  }

  return Stage::RUNNING;
}

Stage::StageStatus
TrafficLightUnprotectedRightTurnStageCreep::FinishScenario() {
  PlanningContext::Instance()->mutable_planning_status()->clear_traffic_light();

  next_stage_ = ScenarioConfig::NO_STAGE;
  return Stage::FINISHED;
}

Stage::StageStatus TrafficLightUnprotectedRightTurnStageCreep::FinishStage() {
  next_stage_ =
      ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE;
  return Stage::FINISHED;
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
