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

#include "modules/planning/scenarios/bare_intersection/unprotected/stage_approach.h"

#include <vector>

#include "cyber/common/log.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace bare_intersection {

using common::TrajectoryPoint;
using hdmap::PathOverlap;

Stage::StageStatus BareIntersectionUnprotectedStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "BareIntersectionUnprotectedStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  const std::string pnc_junction_overlap_id =
      GetContext()->current_pnc_junction_overlap_id;
  if (pnc_junction_overlap_id.empty()) {
    return FinishScenario();
  }

  // get overlap along reference line
  PathOverlap* current_pnc_junction = scenario::util::GetOverlapOnReferenceLine(
      reference_line_info, pnc_junction_overlap_id,
      ReferenceLineInfo::PNC_JUNCTION);
  if (!current_pnc_junction) {
    return FinishScenario();
  }

  constexpr double kPassStopLineBuffer = 0.3;  // unit: m
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_to_pnc_junction =
      current_pnc_junction->start_s - adc_front_edge_s;
  ADEBUG << "pnc_junction_overlap_id[" << pnc_junction_overlap_id
         << "] start_s[" << current_pnc_junction->start_s
         << "] distance_adc_to_pnc_junction[" << distance_adc_to_pnc_junction
         << "]";
  if (distance_adc_to_pnc_junction > kPassStopLineBuffer) {
    // passed stop line
    return FinishStage();
  }

  // set speed_limit to slow down
  if (frame->mutable_reference_line_info()) {
    auto* reference_line =
        frame->mutable_reference_line_info()->front().mutable_reference_line();
    reference_line->AddSpeedLimit(0.0, current_pnc_junction->start_s,
                                  scenario_config_.approach_speed_limit());
  }

  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(current_pnc_junction->start_s,
                                            false);

  plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "BareIntersectionUnprotectedStageApproach planning error";
  }

  bool clear = false;
  // TODO(all): check CLEAR
  if (clear) {
    return FinishStage();
  }

  return Stage::RUNNING;
}

Stage::StageStatus BareIntersectionUnprotectedStageApproach::FinishStage() {
  next_stage_ =
      ScenarioConfig::BARE_INTERSECTION_UNPROTECTED_INTERSECTION_CRUISE;
  return Stage::FINISHED;
}

}  // namespace bare_intersection
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
