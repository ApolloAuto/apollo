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

#include "modules/planning/scenarios/bare_intersection_unprotected/stage_intersection_cruise.h"

#include <memory>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

StageResult BareIntersectionUnprotectedStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "StopSignUnprotectedStageIntersectionCruise plan error";
  }

  bool stage_done = CheckDone(*frame, injector_->planning_context(), false);
  if (stage_done) {
    return FinishStage();
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}

hdmap::PathOverlap*
BareIntersectionUnprotectedStageIntersectionCruise::GetTrafficSignOverlap(
    const ReferenceLineInfo& reference_line_info,
    const PlanningContext* context) const {
  // stop_sign scenarios
  const std::string pnc_junction_overlap_id =
      GetContextAs<BareIntersectionUnprotectedContext>()
          ->current_pnc_junction_overlap_id;
  hdmap::PathOverlap* traffic_sign_overlap =
      reference_line_info.GetOverlapOnReferenceLine(
          pnc_junction_overlap_id, ReferenceLineInfo::PNC_JUNCTION);
  return traffic_sign_overlap;
}

StageResult BareIntersectionUnprotectedStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}

}  // namespace planning
}  // namespace apollo
