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

#include "modules/planning/scenarios/stop_sign/unprotected/stage_intersection_cruise.h"

#include "cyber/common/log.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign {

using common::TrajectoryPoint;

Stage::StageStatus StopSignUnprotectedStageIntersectionCruise::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStageIntersectionCruise plan error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  // set right_of_way_status
  const double stop_sign_start_s =
      PlanningContext::GetScenarioInfo()->current_stop_sign_overlap.start_s;
  reference_line_info.SetJunctionRightOfWay(stop_sign_start_s, false);

  // check pass pnc_junction
  // TODO(all): remove when pnc_junction completely available on map
  const auto& pnc_junction_overlaps =
      reference_line_info.reference_line().map_path().pnc_junction_overlaps();
  if (pnc_junction_overlaps.empty()) {
    // pnc_junction not exist on map, use current stop_sign's end_s

    constexpr double kIntersectionPassDist = 20.0;  // unit: m
    const double adc_back_edge_s =
        reference_line_info.AdcSlBoundary().start_s();
    const double stop_sign_end_s =
        PlanningContext::GetScenarioInfo()->current_stop_sign_overlap.end_s;
    const double distance_adc_pass_stop_sign =
        adc_back_edge_s - stop_sign_end_s;
    ADEBUG << "distance_adc_pass_stop_sign[" << distance_adc_pass_stop_sign
           << "] stop_sign_end_s[" << stop_sign_end_s << "]";

    if (distance_adc_pass_stop_sign >= kIntersectionPassDist) {
      return FinishStage();
    } else {
      return Stage::RUNNING;
    }
  }

  if (!scenario::CheckInsidePnCJunction(reference_line_info)) {
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
