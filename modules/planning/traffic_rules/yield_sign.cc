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

#include "modules/planning/traffic_rules/yield_sign.h"

#include <memory>

#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

YieldSign::YieldSign(const TrafficRuleConfig& config,
                     const std::shared_ptr<DependencyInjector>& injector)
    : TrafficRule(config, injector) {}

Status YieldSign::ApplyRule(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}

void YieldSign::MakeDecisions(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!config_.yield_sign().enabled()) {
    return;
  }

  const auto& yield_sign_status =
      injector_->planning_context()->planning_status().yield_sign();
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();

  const std::vector<PathOverlap>& yield_sign_overlaps =
      reference_line_info->reference_line().map_path().yield_sign_overlaps();
  for (const auto& yield_sign_overlap : yield_sign_overlaps) {
    if (yield_sign_overlap.end_s <= adc_front_edge_s) {
      continue;
    }

    // check if yield-sign-stop already finished, set by scenario/stage
    bool yield_sign_done = false;
    for (const auto& done_yield_sign_overlap_id :
         yield_sign_status.done_yield_sign_overlap_id()) {
      if (yield_sign_overlap.object_id == done_yield_sign_overlap_id) {
        yield_sign_done = true;
        break;
      }
    }
    if (yield_sign_done) {
      continue;
    }

    // build stop decision
    ADEBUG << "BuildStopDecision: yield_sign[" << yield_sign_overlap.object_id
           << "] start_s[" << yield_sign_overlap.start_s << "]";
    const std::string virtual_obstacle_id =
        YIELD_SIGN_VO_ID_PREFIX + yield_sign_overlap.object_id;
    const std::vector<std::string> wait_for_obstacle_ids(
        yield_sign_status.wait_for_obstacle_id().begin(),
        yield_sign_status.wait_for_obstacle_id().end());
    util::BuildStopDecision(virtual_obstacle_id, yield_sign_overlap.start_s,
                            config_.yield_sign().stop_distance(),
                            StopReasonCode::STOP_REASON_YIELD_SIGN,
                            wait_for_obstacle_ids,
                            TrafficRuleConfig::RuleId_Name(config_.rule_id()),
                            frame, reference_line_info);
  }
}

}  // namespace planning
}  // namespace apollo
