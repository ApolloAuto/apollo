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

#include <vector>

#include "modules/planning/scenarios/traffic_light/unprotected_right_turn/stage_stop.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

using common::TrajectoryPoint;
using common::time::Clock;
using perception::TrafficLight;

Stage::StageStatus TrafficLightUnprotectedRightTurnStageStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Stop";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightRightTurnUnprotectedStop planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  bool traffic_light_all_stop = true;
  bool traffic_light_all_green = true;
  for (const auto& traffic_light_overlap :
       PlanningContext::GetScenarioInfo()->current_traffic_light_overlaps) {
    // check if the traffic_light is still along reference_line
    if (scenario::CheckTrafficLightDone(reference_line_info,
                                        traffic_light_overlap.object_id)) {
      continue;
    }

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(
        traffic_light_overlap.start_s, false);

    const double adc_front_edge_s =
        reference_line_info.AdcSlBoundary().end_s();
    const double distance_adc_to_stop_line = traffic_light_overlap.start_s -
        adc_front_edge_s;
    auto signal_color =
        scenario::GetSignal(traffic_light_overlap.object_id).color();
    ADEBUG << "traffic_light_overlap_id[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s
           << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
           << "] color[" << signal_color << "]";

    // check distance to stop line
    if (distance_adc_to_stop_line >
        scenario_config_.max_valid_stop_distance()) {
      traffic_light_all_stop = false;
      break;
    }

    // check on traffic light color
    if (signal_color != TrafficLight::GREEN) {
      traffic_light_all_green = false;
      break;
    }
  }

  if (traffic_light_all_stop && traffic_light_all_green) {
    return FinishStage(true);
  }

  // when right_turn_on_red is enabled
  if (scenario_config_.enable_right_turn_on_red()) {
    // check on wait-time
    if (traffic_light_all_stop && !traffic_light_all_green) {
      if (GetContext()->stop_start_time == 0.0) {
        GetContext()->stop_start_time = Clock::NowInSeconds();
      } else {
        auto start_time = GetContext()->stop_start_time;
        const double wait_time = Clock::NowInSeconds() - start_time;
        ADEBUG << "stop_start_time[" << start_time
               << "] wait_time[" << wait_time << "]";
        if (wait_time > scenario_config_.red_light_right_turn_stop_duration()) {
          return FinishStage(false);
        }
      }
    }
  }

  return Stage::RUNNING;
}

Stage::StageStatus TrafficLightUnprotectedRightTurnStageStop::FinishScenario() {
  PlanningContext::GetScenarioInfo()->stop_done_overlap_ids.clear();

  next_stage_ = ScenarioConfig::NO_STAGE;
  return Stage::FINISHED;
}

Stage::StageStatus TrafficLightUnprotectedRightTurnStageStop::FinishStage(
    const bool protected_mode) {
  if (protected_mode) {
    // intersection_cruise
    next_stage_ = ScenarioConfig ::
        TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE;
  } else {
    // creep
    // update PlanningContext
    PlanningContext::GetScenarioInfo()->stop_done_overlap_ids.clear();
    for (const auto& traffic_light_overlap :
         PlanningContext::GetScenarioInfo()->current_traffic_light_overlaps) {
      PlanningContext::GetScenarioInfo()->stop_done_overlap_ids.push_back(
          traffic_light_overlap.object_id);
    }

    GetContext()->creep_start_time = Clock::NowInSeconds();
    next_stage_ = ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_CREEP;
  }
  return Stage::FINISHED;
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
