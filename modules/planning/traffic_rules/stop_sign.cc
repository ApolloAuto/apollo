/*****************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/traffic_rules/stop_sign.h"

#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {

using common::Status;
using hdmap::PathOverlap;

StopSign::StopSign(const TrafficRuleConfig& config)
    : TrafficRule(config) {}

Status StopSign::ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info) {
  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}

void StopSign::MakeDecisions(
    Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!config_.stop_sign().enabled()) {
    return;
  }

  const double adc_back_edge_s =
      reference_line_info->AdcSlBoundary().start_s();

  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info->reference_line().map_path().stop_sign_overlaps();
  for (const auto& stop_sign_overlap : stop_sign_overlaps) {
    if (stop_sign_overlap.end_s <= adc_back_edge_s) {
      continue;
    }

    const auto& stop_done_overlap_ids =
        PlanningContext::GetScenarioInfo()->stop_done_overlap_ids;
    if (stop_done_overlap_ids.end() !=
        std::find(stop_done_overlap_ids.begin(),
                  stop_done_overlap_ids.end(),
                  stop_sign_overlap.object_id)) {
      continue;
    }

    // build stop decision
    ADEBUG << "BuildStopDecision: stop_sign["
           << stop_sign_overlap.object_id
           << "] start_s[" << stop_sign_overlap.start_s << "]";
    std::string virtual_obstacle_id =
        STOP_SIGN_VO_ID_PREFIX + stop_sign_overlap.object_id;
    BuildStopDecision(
        virtual_obstacle_id,
        stop_sign_overlap.start_s,
        config_.stop_sign().stop_distance(),
        StopReasonCode::STOP_REASON_STOP_SIGN,
        PlanningContext::GetScenarioInfo()->stop_sign_wait_for_obstacles,
        frame, reference_line_info);
  }
}

}  // namespace planning
}  // namespace apollo
