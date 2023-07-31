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

#include "modules/planning/scenarios/bare_intersection_unprotected/bare_intersection_unprotected_scenario.h"

#include <memory>

#include "modules/planning/planning_base/proto/planning_config.pb.h"
#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/scenarios/bare_intersection_unprotected/stage_approach.h"
#include "modules/planning/scenarios/bare_intersection_unprotected/stage_intersection_cruise.h"

namespace apollo {
namespace planning {

bool BareIntersectionUnprotectedScenario::Init(
    std::shared_ptr<DependencyInjector> injector, const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario " << name;
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioBareIntersectionUnprotectedConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get scenario specific config in" << name;
    return false;
  }
  init_ = true;
  return true;
}

/*
 * read scenario specific configs and set in context_ for stages to read
 */

bool BareIntersectionUnprotectedScenario::IsTransferable(
    const Scenario* const other_scenario, const Frame& frame) {
  if (!frame.local_view().planning_command->has_lane_follow_command()) {
    return false;
  }
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;
  }
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  hdmap::PathOverlap* pnc_junction_overlap = nullptr;
  hdmap::PathOverlap* traffic_sign_overlap = nullptr;
  // note: first_encountered_overlaps already sorted
  if (first_encountered_overlaps.empty()) {
    return false;
  }

  for (const auto& overlap : first_encountered_overlaps) {
    if ((overlap.first == ReferenceLineInfo::SIGNAL ||
         overlap.first == ReferenceLineInfo::STOP_SIGN ||
         overlap.first == ReferenceLineInfo::YIELD_SIGN) &&
        traffic_sign_overlap == nullptr) {
      traffic_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
      break;
    } else if (overlap.first == ReferenceLineInfo::PNC_JUNCTION &&
               pnc_junction_overlap == nullptr) {
      pnc_junction_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
    }
  }

  if (traffic_sign_overlap && pnc_junction_overlap) {
    static constexpr double kJunctionDelta = 10.0;
    double s_diff = std::fabs(traffic_sign_overlap->start_s -
                              pnc_junction_overlap->start_s);
    if (s_diff >= kJunctionDelta) {
      if (pnc_junction_overlap->start_s > traffic_sign_overlap->start_s) {
        pnc_junction_overlap = nullptr;
      } else {
        traffic_sign_overlap = nullptr;
      }
    }
  }

  if (traffic_sign_overlap || !pnc_junction_overlap) {
    return false;
  }

  if (reference_line_info.GetIntersectionRightofWayStatus(
          *pnc_junction_overlap)) {
    return false;
  }

  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_pnc_junction =
      pnc_junction_overlap->start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_pnc_junction[" << adc_distance_to_pnc_junction
         << "] pnc_junction[" << pnc_junction_overlap->object_id
         << "] pnc_junction_overlap_start_s[" << pnc_junction_overlap->start_s
         << "]";

  const bool bare_junction_scenario =
      (adc_distance_to_pnc_junction > 0.0 &&
       adc_distance_to_pnc_junction <=
           context_.scenario_config
               .start_bare_intersection_scenario_distance());

  return bare_junction_scenario;
}

bool BareIntersectionUnprotectedScenario::Enter(Frame* frame) {
  // set to first_encountered pnc_junction
  const auto& first_encountered_overlaps =
      frame->reference_line_info().front().FirstEncounteredOverlaps();
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::PNC_JUNCTION) {
      context_.current_pnc_junction_overlap_id = overlap.second.object_id;
      ADEBUG << "Update PlanningContext with first_encountered pnc_junction["
             << overlap.second.object_id << "] start_s["
             << overlap.second.start_s << "]";
      break;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
