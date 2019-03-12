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
#include <string>
#include <vector>

#include "modules/planning/scenarios/stop_sign/unprotected/stage_creep.h"

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/tasks/deciders/decider_creep.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign {

using common::TrajectoryPoint;
using common::time::Clock;
using hdmap::PathOverlap;

Stage::StageStatus StopSignUnprotectedStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    return FinishStage();
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStageCreep planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  // check if the stop_sign is still along reference_line
  std::string stop_sign_overlap_id =
      PlanningContext::GetScenarioInfo()->current_stop_sign_overlap.object_id;
  if (scenario::CheckStopSignDone(reference_line_info, stop_sign_overlap_id)) {
    return FinishScenario();
  }

  // set right_of_way_status
  const double stop_sign_start_s =
      PlanningContext::GetScenarioInfo()->current_stop_sign_overlap.start_s;
  reference_line_info.SetJunctionRightOfWay(stop_sign_start_s, false);

  const double stop_sign_end_s =
      PlanningContext::GetScenarioInfo()->current_stop_sign_overlap.end_s;
  const double wait_time =
      Clock::NowInSeconds() - GetContext()->creep_start_time;
  const double timeout = scenario_config_.creep_timeout();
  auto* task = dynamic_cast<DeciderCreep*>(FindTask(TaskConfig::DECIDER_CREEP));
  if (task && task->CheckCreepDone(*frame, reference_line_info, stop_sign_end_s,
                                   wait_time, timeout)) {
    return FinishStage();
  }

  // set param for PROCEED_WITH_CAUTION_SPEED
  dynamic_cast<DeciderCreep*>(FindTask(TaskConfig::DECIDER_CREEP))
      ->SetProceedWithCautionSpeedParam(*frame, reference_line_info,
                                        stop_sign_end_s);

  plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStageCreep planning error";
  }
  return Stage::RUNNING;
}

Stage::StageStatus StopSignUnprotectedStageCreep::FinishStage() {
  next_stage_ = ScenarioConfig::STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE;
  return Stage::FINISHED;
}

}  // namespace stop_sign
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
