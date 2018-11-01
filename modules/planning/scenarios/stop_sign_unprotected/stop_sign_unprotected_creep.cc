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

#include "modules/planning/scenarios/stop_sign_unprotected/stop_sign_unprotected_creep.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/toolkits/deciders/decider_creep.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign_protected {

using common::TrajectoryPoint;

Stage::StageStatus StopSignUnprotectedCreep::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);

  if (!config_.enabled()) {
    next_stage_ = ScenarioConfig::STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE;
    return Stage::FINISHED;
  }

  auto& reference_line_info = frame->mutable_reference_line_info()->front();
  const double stop_sign_overlap_end_s =
      PlanningContext::GetScenarioInfo()->next_stop_sign_overlap.end_s;
  if (dynamic_cast<DeciderCreep*>(FindTask(TaskConfig::DECIDER_CREEP))
          ->CheckCreepDone(*frame, reference_line_info,
                           stop_sign_overlap_end_s)) {
    return Stage::FINISHED;
  }

  // set param for PROCEED_WITH_CAUTION_SPEED
  dynamic_cast<DeciderCreep*>(FindTask(TaskConfig::DECIDER_CREEP))
      ->SetProceedWithCautionSpeedParam(*frame, reference_line_info,
                                        stop_sign_overlap_end_s);

  bool plan_ok = PlanningOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedCreep planning error";
  }
  return Stage::RUNNING;
}

}  // namespace stop_sign_protected
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
