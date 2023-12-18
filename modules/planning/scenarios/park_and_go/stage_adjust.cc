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

#include "modules/planning/scenarios/park_and_go/stage_adjust.h"

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/scenarios/park_and_go/context.h"
#include "modules/planning/scenarios/park_and_go/util.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

StageResult ParkAndGoStageAdjust::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Adjust";
  CHECK_NOTNULL(frame);

  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  StageResult result = ExecuteTaskOnOpenSpace(frame);
  if (result.HasError()) {
    AERROR << "ParkAndGoStageAdjust planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }
  const bool is_ready_to_cruise =
      CheckADCReadyToCruise(injector_->vehicle_state(), frame,
                            GetContextAs<ParkAndGoContext>()->scenario_config);

  bool is_end_of_trajectory = false;
  const auto& history_frame = injector_->frame_history()->Latest();
  if (history_frame) {
    const auto& trajectory_points =
        history_frame->current_frame_planned_trajectory().trajectory_point();
    if (!trajectory_points.empty()) {
      is_end_of_trajectory =
          (trajectory_points.rbegin()->relative_time() < 0.0);
    }
  }

  if (!is_ready_to_cruise && !is_end_of_trajectory) {
    return result.SetStageStatus(StageStatusType::RUNNING);
  }
  return FinishStage();
}

StageResult ParkAndGoStageAdjust::FinishStage() {
  const auto vehicle_status = injector_->vehicle_state();
  ADEBUG << vehicle_status->steering_percentage();
  if (std::fabs(vehicle_status->steering_percentage()) <
      GetContextAs<ParkAndGoContext>()
          ->scenario_config.max_steering_percentage_when_cruise()) {
    next_stage_ = "PARK_AND_GO_CRUISE";
  } else {
    ResetInitPostion();
    next_stage_ = "PARK_AND_GO_PRE_CRUISE";
  }
  return StageResult(StageStatusType::FINISHED);
}

void ParkAndGoStageAdjust::ResetInitPostion() {
  auto* park_and_go_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_park_and_go();
  park_and_go_status->mutable_adc_init_position()->set_x(
      injector_->vehicle_state()->x());
  park_and_go_status->mutable_adc_init_position()->set_y(
      injector_->vehicle_state()->y());
  park_and_go_status->mutable_adc_init_position()->set_z(0.0);
  park_and_go_status->set_adc_init_heading(
      injector_->vehicle_state()->heading());
}

}  // namespace planning
}  // namespace apollo
