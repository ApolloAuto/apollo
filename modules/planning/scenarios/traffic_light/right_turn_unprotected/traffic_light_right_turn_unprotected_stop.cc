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

#include "modules/planning/scenarios/traffic_light/right_turn_unprotected/traffic_light_right_turn_unprotected_stop.h"

#include "modules/perception/proto/perception_obstacle.pb.h"

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

Stage::StageStatus TrafficLightRightTurnUnprotectedStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Stop";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  auto& reference_line_info = frame->mutable_reference_line_info()->front();

  bool waiting = false;
  const std::string traffic_light_id =
      PlanningContext::GetScenarioInfo()->next_traffic_light_overlap.object_id;
  for (const auto* obstacle :
       reference_line_info.path_decision()->obstacles().Items()) {
    if (obstacle->Id() != traffic_light_id) {
      continue;
    }
    if (obstacle->LongitudinalDecision().has_stop() &&
        obstacle->LongitudinalDecision().stop().reason_code() ==
            STOP_REASON_SIGNAL) {
      waiting = true;
      break;
    }
  }

  if (waiting) {
    return Stage::RUNNING;
  }

  next_stage_ = ScenarioConfig::TRAFFIC_LIGHT_RIGHT_TURN_UNPROTECTED_CREEP;
  PlanningContext::GetScenarioInfo()->stop_done_overlap_id =
      GetContext()->traffic_light_id;
  GetContext()->creep_start_time = Clock::NowInSeconds();
  return Stage::FINISHED;
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
