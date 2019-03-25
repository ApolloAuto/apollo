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

#include "modules/planning/scenarios/traffic_light/unprotected_left_turn/stage_intersection_cruise.h"

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
using hdmap::PathOverlap;

Stage::StageStatus
TrafficLightUnprotectedLeftTurnStageIntersectionCruise::Process(

    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);
  
  if (!config_.enabled()) {
    ADEBUG << "stage IntersectionCruise finished because not enable.";
    return FinishStage();
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageIntersectionCruise "
           << "plan error";
  }

  /* TODO(all): to be fixed
  const auto& reference_line_info = frame->reference_line_info().front();

  // check if the traffic_light is still along reference_line
  std::string traffic_light_overlap_id =
      PlanningContext::GetScenarioInfo()->next_traffic_light_overlap.object_id;
  if (CheckTrafficLightDone(reference_line_info, traffic_light_overlap_id)) {
    return FinishScenario();
  }

  // check pass intersection
  // TODO(all): update when pnc-junction is ready
  constexpr double kIntersectionLength = 10.0;  // unit: m
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();
  const double distance_adc_pass_traffic_light =
      adc_back_edge_s -
      PlanningContext::GetScenarioInfo()->next_traffic_light_overlap.end_s;
  if (distance_adc_pass_traffic_light > kIntersectionLength) {
    return FinishStage();
  }
  */

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
