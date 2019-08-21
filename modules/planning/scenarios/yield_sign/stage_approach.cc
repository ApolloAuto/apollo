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

#include <vector>

#include "modules/planning/scenarios/yield_sign/stage_approach.h"

#include "cyber/common/log.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace yield_sign {

using apollo::common::TrajectoryPoint;
using apollo::hdmap::PathOverlap;
using apollo::perception::TrafficLight;

Stage::StageStatus YieldSignStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
   << "stage: Approach";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "YieldSignStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  const std::string yield_sign_overlap_id =
      GetContext()->current_yield_sign_overlap_id;
  // refresh overlap along reference line
  PathOverlap* current_yield_sign_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                yield_sign_overlap_id,
                                                ReferenceLineInfo::YIELD_SIGN);
  if (!current_yield_sign_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(
      current_yield_sign_overlap->start_s, false);

  bool yield_sign_clear = false;
  // check distance to stop line
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_to_stop_line =
      current_yield_sign_overlap->start_s - adc_front_edge_s;
   << "yield_sign_overlap_id[" << yield_sign_overlap_id
         << "] start_s[" << current_yield_sign_overlap->start_s
         << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line << "]";
  if (distance_adc_to_stop_line <
      scenario_config_.max_valid_stop_distance()) {
    // TODO(all): check YIELD_SIGN clear
    yield_sign_clear = true;
  }

  if (yield_sign_clear) {
    return FinishStage();
  }

  return Stage::RUNNING;
}

Stage::StageStatus YieldSignStageApproach::FinishScenario() {
  PlanningContext::Instance()->mutable_planning_status()->clear_traffic_light();

  next_stage_ = ScenarioConfig::NO_STAGE;
  return Stage::FINISHED;
}

Stage::StageStatus YieldSignStageApproach::FinishStage() {
  next_stage_ = ScenarioConfig::YIELD_SIGN_CREEP;
  return Stage::FINISHED;
}

}  // namespace yield_sign
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
