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

#include "modules/planning/scenarios/traffic_light/right_turn_unprotected/traffic_light_right_turn_unprotected_pre_stop.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/toolkits/deciders/decider_creep.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

using common::TrajectoryPoint;
using common::time::Clock;
using hdmap::HDMapUtil;
using hdmap::LaneInfo;
using hdmap::LaneInfoConstPtr;
using hdmap::OverlapInfoConstPtr;
using perception::PerceptionObstacle;

Stage::StageStatus TrafficLightRightTurnUnprotectedPreStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: PreStop";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  const auto& reference_line_info = frame->reference_line_info().front();
  if (CheckADCStop(reference_line_info)) {
    next_stage_ = ScenarioConfig::STOP_SIGN_UNPROTECTED_STOP;
    return Stage::FINISHED;
  }

  bool plan_ok = PlanningOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightRightTurnUnprotectedPreStop planning error";
  }
  return Stage::RUNNING;
}

/**
 * @brief: check valid traffic_light stop
 */
bool TrafficLightRightTurnUnprotectedPreStop::CheckADCStop(
    const ReferenceLineInfo& reference_line_info) {
  double adc_speed =
      common::VehicleStateProvider::Instance()->linear_velocity();
  if (adc_speed > scenario_config_.max_adc_stop_speed()) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;
  }

  // check stop close enough to stop line of the traffic_light
  double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  double stop_line_start_s =
      PlanningContext::GetScenarioInfo()->next_traffic_light_overlap.start_s;
  double distance_stop_line_to_adc_front_edge =
      stop_line_start_s - adc_front_edge_s;
  ADEBUG << "distance_stop_line_to_adc_front_edge["
         << distance_stop_line_to_adc_front_edge << "]; stop_line_start_s["
         << stop_line_start_s << "]; adc_front_edge_s[" << adc_front_edge_s
         << "]";

  if (distance_stop_line_to_adc_front_edge >
      scenario_config_.max_valid_stop_distance()) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;
  }

  // TODO(all): check no BICYCLE in between.

  return true;
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
