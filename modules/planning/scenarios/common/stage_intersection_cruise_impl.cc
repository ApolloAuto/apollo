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

#include "modules/planning/scenarios/common/stage_intersection_cruise_impl.h"

#include <string>

#include "cyber/common/log.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {
namespace scenario {

bool StageIntersectionCruiseImpl::CheckDone(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type,
    const ScenarioConfig::StageConfig& config, const bool right_of_way_status) {
  const auto& reference_line_info = frame.reference_line_info().front();

  const auto& pnc_junction_overlaps =
      reference_line_info.reference_line().map_path().pnc_junction_overlaps();
  if (pnc_junction_overlaps.empty()) {
    // TODO(all): remove when pnc_junction completely available on map
    // pnc_junction not exist on map, use current traffic_sign's end_s
    // get traffic sign overlap along reference line
    hdmap::PathOverlap* traffic_sign_overlap = nullptr;
    if (scenario_type == ScenarioConfig::STOP_SIGN_PROTECTED ||
        scenario_type == ScenarioConfig::STOP_SIGN_UNPROTECTED) {
      // stop_sign scenarios
      const auto& stop_sign_status =
          PlanningContext::Instance()->planning_status().stop_sign();
      const std::string traffic_sign_overlap_id =
          stop_sign_status.current_stop_sign_overlap_id();
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::STOP_SIGN);
    } else if (scenario_type == ScenarioConfig::TRAFFIC_LIGHT_PROTECTED ||
               scenario_type ==
                   ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN ||
               scenario_type ==
                   ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN) {
      // traffic_light scenarios
      const auto& traffic_light_status =
          PlanningContext::Instance()->planning_status().traffic_light();
      const std::string traffic_sign_overlap_id =
          traffic_light_status.current_traffic_light_overlap_id_size() > 0
              ? traffic_light_status.current_traffic_light_overlap_id(0)
              : "";
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::SIGNAL);
    } else if (scenario_type == ScenarioConfig::YIELD_SIGN) {
      // yield_sign scenarios
      const auto& yield_sign_status =
          PlanningContext::Instance()->planning_status().yield_sign();
      const std::string traffic_sign_overlap_id =
          yield_sign_status.current_yield_sign_overlap_id_size() > 0
              ? yield_sign_status.current_yield_sign_overlap_id(0)
              : "";
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::YIELD_SIGN);
    }

    if (!traffic_sign_overlap) {
      return true;
    }

    static constexpr double kIntersectionPassDist = 20.0;  // unit: m
    const double adc_back_edge_s =
        reference_line_info.AdcSlBoundary().start_s();
    const double distance_adc_pass_traffic_sign =
        adc_back_edge_s - traffic_sign_overlap->end_s;
    ADEBUG << "distance_adc_pass_traffic_sign["
           << distance_adc_pass_traffic_sign << "] traffic_sign_end_s["
           << traffic_sign_overlap->end_s << "]";

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(traffic_sign_overlap->start_s,
                                              right_of_way_status);

    return distance_adc_pass_traffic_sign >= kIntersectionPassDist;
  }

  if (!planning::util::CheckInsidePnCJunction(reference_line_info)) {
    return true;
  }

  // set right_of_way_status
  hdmap::PathOverlap pnc_junction_overlap;
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  reference_line_info.GetPnCJunction(adc_front_edge_s, &pnc_junction_overlap);
  reference_line_info.SetJunctionRightOfWay(pnc_junction_overlap.start_s,
                                            right_of_way_status);

  return false;
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
