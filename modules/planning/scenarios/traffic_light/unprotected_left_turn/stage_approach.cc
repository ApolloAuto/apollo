/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/traffic_light/unprotected_left_turn/stage_approach.h"

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/tasks/deciders/creep_decider/creep_decider.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

using apollo::common::TrajectoryPoint;
using apollo::common::time::Clock;
using apollo::hdmap::PathOverlap;
using apollo::perception::TrafficLight;

Stage::StageStatus TrafficLightUnprotectedLeftTurnStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    return FinishStage(frame);
  }

  // set cruise_speed to slow down
  frame->mutable_reference_line_info()->front().SetCruiseSpeed(
      scenario_config_.approach_cruise_speed());

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageApproach planning error";
  }

  if (GetContext()->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  PathOverlap* traffic_light = nullptr;
  bool traffic_light_all_done = true;
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

    traffic_light = current_traffic_light_overlap;

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(
        current_traffic_light_overlap->start_s, false);

    const double distance_adc_to_stop_line =
        current_traffic_light_overlap->start_s - adc_front_edge_s;
    auto signal_color = frame->GetSignal(traffic_light_overlap_id).color();
    ADEBUG << "traffic_light_overlap_id[" << traffic_light_overlap_id
           << "] start_s[" << current_traffic_light_overlap->start_s
           << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
           << "] color[" << signal_color << "]";

    // check on traffic light color and distance to stop line
    if (signal_color != TrafficLight::GREEN ||
        distance_adc_to_stop_line >=
            scenario_config_.max_valid_stop_distance()) {
      traffic_light_all_done = false;
      break;
    }
  }

  if (traffic_light == nullptr) {
    return FinishScenario();
  }

  if (traffic_light_all_done) {
    return FinishStage(frame);
  }

  return Stage::RUNNING;
}

Stage::StageStatus TrafficLightUnprotectedLeftTurnStageApproach::FinishStage(
    Frame* frame) {
  // check speed at stop_stage
  const double adc_speed =
      common::VehicleStateProvider::Instance()->linear_velocity();
  if (adc_speed > scenario_config_.max_adc_speed_before_creep()) {
    // skip creep
    next_stage_ = ScenarioConfig ::
        TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE;
  } else {
    // creep
    // update PlanningContext
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->mutable_done_traffic_light_overlap_id()
        ->Clear();
    for (const auto& traffic_light_overlap_id :
         GetContext()->current_traffic_light_overlap_ids) {
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->mutable_traffic_light()
          ->add_done_traffic_light_overlap_id(traffic_light_overlap_id);
    }

    GetContext()->creep_start_time = Clock::NowInSeconds();
    next_stage_ = ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP;
  }

  // reset cruise_speed
  auto& reference_line_info = frame->mutable_reference_line_info()->front();
  reference_line_info.SetCruiseSpeed(FLAGS_default_cruise_speed);

  return Stage::FINISHED;
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
