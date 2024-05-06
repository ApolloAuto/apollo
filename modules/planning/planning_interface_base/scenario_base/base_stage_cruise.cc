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
 * @file base_stage_cruise.cc
 **/

#include "modules/planning/planning_interface_base/scenario_base/base_stage_cruise.h"

#include <string>

#include "cyber/common/log.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/util.h"

namespace apollo {
namespace planning {

bool BaseStageCruise::CheckDone(const Frame& frame,
                                const PlanningContext* context,
                                const bool right_of_way_status) {
  const auto& reference_line_info = frame.reference_line_info().front();

  const auto& junction_overlaps =
      reference_line_info.reference_line().map_path().junction_overlaps();
  if (junction_overlaps.empty()) {
    // TODO(all): remove when pnc_junction completely available on map
    // pnc_junction not exist on map, use current traffic_sign's end_s
    // get traffic sign overlap along reference line
    hdmap::PathOverlap* traffic_sign_overlap =
        GetTrafficSignOverlap(reference_line_info, context);
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

  if (!planning::util::CheckInsideJunction(reference_line_info)) {
    return true;
  }

  // set right_of_way_status
  hdmap::PathOverlap junction_overlap;
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  reference_line_info.GetJunction(adc_front_edge_s, &junction_overlap);
  reference_line_info.SetJunctionRightOfWay(junction_overlap.start_s,
                                            right_of_way_status);

  return false;
}

}  // namespace planning
}  // namespace apollo
