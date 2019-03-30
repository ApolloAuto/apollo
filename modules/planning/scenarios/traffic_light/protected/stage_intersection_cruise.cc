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

#include "modules/planning/scenarios/traffic_light/protected/stage_intersection_cruise.h"

#include "cyber/common/log.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/util/util.h"

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

  if (GetContext()->current_traffic_light_overlap_ids.size() <= 0) {
    return FinishStage();
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  // get overlap along reference line
  PathOverlap* current_traffic_light_overlap =
        scenario::util::GetOverlapOnReferenceLine(
            reference_line_info,
            GetContext()->current_traffic_light_overlap_ids[0],
            ReferenceLineInfo::SIGNAL);
  if (!current_traffic_light_overlap) {
    return FinishStage();
  }

  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(
      current_traffic_light_overlap->start_s, true);

  // check pass pnc_junction
  // TODO(all): remove when pnc_junction completely available on map
  const auto& pnc_junction_overlaps =
      reference_line_info.reference_line().map_path().pnc_junction_overlaps();
  if (pnc_junction_overlaps.empty()) {
    // pnc_junction not exist on map, use current traffic_light's end_s
    constexpr double kIntersectionPassDist = 20.0;  // unit: m
    const double adc_back_edge_s =
        reference_line_info.AdcSlBoundary().start_s();
    const double distance_adc_pass_traffic_light =
        adc_back_edge_s - current_traffic_light_overlap->end_s;
    ADEBUG << "distance_adc_pass_traffic_light["
           << distance_adc_pass_traffic_light << "] traffic_light_end_s["
           << current_traffic_light_overlap->end_s << "]";

    if (distance_adc_pass_traffic_light >= kIntersectionPassDist) {
      return FinishStage();
    } else {
      return Stage::RUNNING;
    }
  }

  if (!planning::util::CheckInsidePnCJunction(reference_line_info)) {
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
