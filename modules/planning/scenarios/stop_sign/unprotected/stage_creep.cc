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

#include "modules/planning/scenarios/stop_sign/unprotected/stage_creep.h"

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/tasks/deciders/creep_decider/creep_decider.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign {

using apollo::common::TrajectoryPoint;
using apollo::common::time::Clock;
using apollo::hdmap::PathOverlap;

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

  std::string stop_sign_overlap_id = GetContext()->current_stop_sign_overlap_id;

  // get overlap along reference line
  PathOverlap* current_stop_sign_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                stop_sign_overlap_id,
                                                ReferenceLineInfo::STOP_SIGN);
  if (!current_stop_sign_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  const double stop_sign_start_s = current_stop_sign_overlap->start_s;
  reference_line_info.SetJunctionRightOfWay(stop_sign_start_s, false);

  const double stop_sign_end_s = current_stop_sign_overlap->end_s;
  const double wait_time =
      Clock::NowInSeconds() - GetContext()->creep_start_time;
  const double timeout_sec = scenario_config_.creep_timeout_sec();
  auto* task = dynamic_cast<CreepDecider*>(FindTask(TaskConfig::CREEP_DECIDER));

  if (task == nullptr) {
    AERROR << "task is nullptr";
    return FinishStage();
  }

  double creep_stop_s =
      stop_sign_end_s + task->FindCreepDistance(*frame, reference_line_info);
  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance <= 0.0) {
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }

  if (task->CheckCreepDone(*frame, reference_line_info, stop_sign_end_s,
                           wait_time, timeout_sec)) {
    return FinishStage();
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
