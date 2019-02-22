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

#include "modules/planning/scenarios/traffic_light/protected/stage_intersection_cruise.h"

#include "cyber/common/log.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

using common::TrajectoryPoint;
using hdmap::PathOverlap;

Stage::StageStatus TrafficLightProtectedStageIntersectionCruise::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightProtectedStageIntersectionCruise plan error";
  }

  // check pass intersection
  constexpr double kIntersectionLength = 2.0;  // unit: m
  const auto& reference_line_info = frame->reference_line_info().front();
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();
  const double distance_adc_pass_traffic_light = adc_back_edge_s -
      PlanningContext::GetScenarioInfo()->next_pnc_junction_overlap.end_s;
  ADEBUG << "distance_adc_pass_traffic_light["
      << distance_adc_pass_traffic_light << "]";
  if (distance_adc_pass_traffic_light >= kIntersectionLength) {
    return FinishStage();
  }

  return Stage::RUNNING;
}

Stage::StageStatus TrafficLightProtectedStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
