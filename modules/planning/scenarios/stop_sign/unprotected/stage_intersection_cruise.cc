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

#include "modules/planning/scenarios/stop_sign/unprotected/stage_intersection_cruise.h"

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/tasks/deciders/decider_creep.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign {

using common::TrajectoryPoint;
using hdmap::PathOverlap;

Stage::StageStatus StopSignUnprotectedStageIntersectionCruise::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStageIntersectionCruise plan error";
  }

  // check pass pnc_junction
  const auto& reference_line_info = frame->reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();

  hdmap::PathOverlap pnc_junction_overlap;
  reference_line_info.GetPnCJunction(adc_front_edge_s, &pnc_junction_overlap);
  if (pnc_junction_overlap.object_id.empty()) {
    return FinishStage();
  }

  constexpr double kIntersectionPassDist = 2.0;  // unit: m
  const double distance_adc_pass_intersection = adc_back_edge_s -
      pnc_junction_overlap.end_s;
  ADEBUG << "distance_adc_pass_intersection["
      << distance_adc_pass_intersection
      << "] pnc_junction_overlap[" << pnc_junction_overlap.object_id
      << "] start_s[" << pnc_junction_overlap.start_s << "]";
  if (distance_adc_pass_intersection >= kIntersectionPassDist) {
    return FinishStage();
  }

  return Stage::RUNNING;
}

Stage::StageStatus StopSignUnprotectedStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}

}  // namespace stop_sign
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
