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
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/tasks/deciders/decider_creep.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

using common::time::Clock;
using common::TrajectoryPoint;
using hdmap::PathOverlap;
using perception::TrafficLight;

Stage::StageStatus StageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    return FinishStage();
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StageCreep planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  // check if the traffic_light is still along referenceline
  std::string traffic_light_overlap_id = GetContext()->traffic_light_id;
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_it =
      std::find_if(traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
                   [&traffic_light_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == traffic_light_overlap_id;
                   });
  if (traffic_light_overlap_it == traffic_light_overlaps.end()) {
    return FinishScenario();
  }

  // check on traffic light color
  if (PlanningContext::GetScenarioInfo()->traffic_light_color ==
      TrafficLight::GREEN) {
    return FinishStage();
  }

  const double wait_time =
      Clock::NowInSeconds() - GetContext()->creep_start_time;
  const double timeout = scenario_config_.creep_timeout();
  auto *task = dynamic_cast<DeciderCreep*>(FindTask(TaskConfig::DECIDER_CREEP));
  if (task && task->CheckCreepDone(*frame, reference_line_info,
                                   traffic_light_overlap_it->end_s,
                                   wait_time, timeout)) {
    return FinishStage();
  }

  // set param for PROCEED_WITH_CAUTION_SPEED
  dynamic_cast<DeciderCreep*>(FindTask(TaskConfig::DECIDER_CREEP))
      ->SetProceedWithCautionSpeedParam(*frame, reference_line_info,
                                        traffic_light_overlap_it->end_s);

  plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StageCreep planning error";
  }
  return Stage::RUNNING;
}

Stage::StageStatus StageCreep::FinishStage() {
  next_stage_ = ScenarioConfig::STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE;
  return Stage::FINISHED;
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
