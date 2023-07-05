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

#include "modules/planning/scenarios/traffic_light/unprotected_right_turn/stage_stop.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::PathOverlap;
using apollo::perception::TrafficLight;

Stage::StageStatus TrafficLightUnprotectedRightTurnStageStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Stop";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightRightTurnUnprotectedStop planning error";
  }

  if (GetContext()->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  bool traffic_light_all_stop = true;
  bool traffic_light_all_green = true;
  bool traffic_light_no_right_turn_on_red = false;
  PathOverlap* current_traffic_light_overlap = nullptr;

  for (const auto& traffic_light_overlap_id :
       GetContext()->current_traffic_light_overlap_ids) {
    // get overlap along reference line
    current_traffic_light_overlap = scenario::util::GetOverlapOnReferenceLine(
        reference_line_info, traffic_light_overlap_id,
        ReferenceLineInfo::SIGNAL);
    if (!current_traffic_light_overlap) {
      continue;
    }

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

    // check distance to stop line
    if (distance_adc_to_stop_line >
        scenario_config_.max_valid_stop_distance()) {
      traffic_light_all_stop = false;
      break;
    }

    // check on traffic light color
    if (signal_color != TrafficLight::GREEN) {
      traffic_light_all_green = false;
      traffic_light_no_right_turn_on_red =
          CheckTrafficLightNoRightTurnOnRed(traffic_light_overlap_id);
      break;
    }
  }

  if (traffic_light_all_stop && traffic_light_all_green) {
    return FinishStage(true);
  }

  if (!traffic_light_no_right_turn_on_red) {
    if (traffic_light_all_stop && !traffic_light_all_green) {
      // check distance pass stop line
      const double distance_adc_pass_stop_line =
          adc_front_edge_s - current_traffic_light_overlap->end_s;
      ADEBUG << "distance_adc_pass_stop_line[" << distance_adc_pass_stop_line
             << "]";
      if (distance_adc_pass_stop_line >
          scenario_config_.min_pass_s_distance()) {
        return FinishStage(false);
      }

      if (scenario_config_.enable_right_turn_on_red()) {
        // when right_turn_on_red is enabled
        // check on wait-time
        if (GetContext()->stop_start_time == 0.0) {
          GetContext()->stop_start_time = Clock::NowInSeconds();
        } else {
          auto start_time = GetContext()->stop_start_time;
          const double wait_time = Clock::NowInSeconds() - start_time;
          ADEBUG << "stop_start_time[" << start_time << "] wait_time["
                 << wait_time << "]";
          if (wait_time >
              scenario_config_.red_light_right_turn_stop_duration_sec()) {
            return FinishStage(false);
          }
        }
      }
    }
  }

  return Stage::RUNNING;
}

bool TrafficLightUnprotectedRightTurnStageStop::
    CheckTrafficLightNoRightTurnOnRed(const std::string& traffic_light_id) {
  hdmap::SignalInfoConstPtr traffic_light_ptr =
      HDMapUtil::BaseMap().GetSignalById(hdmap::MakeMapId(traffic_light_id));
  if (!traffic_light_ptr) {
    return false;
  }

  const auto& signal = traffic_light_ptr->signal();
  for (int i = 0; i < signal.sign_info_size(); i++) {
    if (signal.sign_info(i).type() == hdmap::SignInfo::NO_RIGHT_TURN_ON_RED) {
      return true;
    }
  }

  return false;
}

Stage::StageStatus TrafficLightUnprotectedRightTurnStageStop::FinishStage(
    const bool protected_mode) {
  if (protected_mode) {
    // intersection_cruise
    next_stage_ = StageType ::
        TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE;
  } else {
    // check speed at stop_stage
    const double adc_speed = injector_->vehicle_state()->linear_velocity();
    if (adc_speed > scenario_config_.max_adc_speed_before_creep()) {
      // skip creep
      next_stage_ = StageType ::
          TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE;
    } else {
      // creep
      // update PlanningContext
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_traffic_light()
          ->mutable_done_traffic_light_overlap_id()
          ->Clear();
      for (const auto& traffic_light_overlap_id :
           GetContext()->current_traffic_light_overlap_ids) {
        injector_->planning_context()
            ->mutable_planning_status()
            ->mutable_traffic_light()
            ->add_done_traffic_light_overlap_id(traffic_light_overlap_id);
      }

      GetContext()->creep_start_time = Clock::NowInSeconds();
      next_stage_ = StageType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_CREEP;
    }
  }
  return Stage::FINISHED;
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
