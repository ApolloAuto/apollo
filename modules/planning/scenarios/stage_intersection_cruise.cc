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

#include "modules/planning/scenarios/stage_intersection_cruise.h"

#include <string>

#include "cyber/common/log.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {
namespace scenario {

using common::TrajectoryPoint;
using hdmap::PathOverlap;

Stage::StageStatus StageIntersectionCruise::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "scenario[" << ScenarioConfig::ScenarioType_Name(scenario_type_)
         << "] stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StageIntersectionCruise plan error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  // TODO(all): remove when pnc_junction completely available on map
  // get traffic sign overlap along reference line
  PathOverlap* traffic_sign_overlap = nullptr;
  if (scenario_type_ == ScenarioConfig::STOP_SIGN_PROTECTED ||
      scenario_type_ == ScenarioConfig::STOP_SIGN_UNPROTECTED) {
    const auto& stop_sign_status =
        PlanningContext::Planningstatus().stop_sign();
    const std::string traffic_sign_overlap_id =
        stop_sign_status.current_stop_sign_overlap_id();
    traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
        reference_line_info,
        traffic_sign_overlap_id,
        ReferenceLineInfo::STOP_SIGN);
  } else if (scenario_type_ == ScenarioConfig::TRAFFIC_LIGHT_PROTECTED ||
      scenario_type_ == ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN ||
      scenario_type_ == ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN) {
    const auto& traffic_light_status =
        PlanningContext::Planningstatus().traffic_light();
    const std::string traffic_sign_overlap_id =
        traffic_light_status.current_traffic_light_overlap_id_size() > 0 ?
            traffic_light_status.current_traffic_light_overlap_id(0) : "";
    traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
        reference_line_info,
        traffic_sign_overlap_id,
        ReferenceLineInfo::SIGNAL);
  }
  if (!traffic_sign_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(traffic_sign_overlap->start_s,
                                            right_of_way_status_);

  // check pass pnc_junction
  // TODO(all): remove when pnc_junction completely available on map
  const auto& pnc_junction_overlaps =
      reference_line_info.reference_line().map_path().pnc_junction_overlaps();
  if (pnc_junction_overlaps.empty()) {
    // pnc_junction not exist on map, use current traffic_sign's end_s
    constexpr double kIntersectionPassDist = 20.0;  // unit: m
    const double adc_back_edge_s =
        reference_line_info.AdcSlBoundary().start_s();
    const double distance_adc_pass_traffic_sign =
        adc_back_edge_s - traffic_sign_overlap->end_s;
    ADEBUG << "distance_adc_pass_traffic_sign["
           << distance_adc_pass_traffic_sign
           << "] traffic_sign_end_s[" << traffic_sign_overlap->end_s << "]";

    if (distance_adc_pass_traffic_sign >= kIntersectionPassDist) {
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

Stage::StageStatus StageIntersectionCruise::FinishStage() {
  return FinishScenario();
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
