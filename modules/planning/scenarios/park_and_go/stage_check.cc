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

#include "modules/planning/scenarios/park_and_go/stage_check.h"

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/scenarios/park_and_go/context.h"
#include "modules/planning/scenarios/park_and_go/util.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

StageResult ParkAndGoStageCheck::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Check";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  ADCInitStatus();
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  StageResult result = ExecuteTaskOnOpenSpace(frame);
  if (result.HasError()) {
    AERROR << "ParkAndGoStageAdjust planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  bool ready_to_cruise =
      CheckADCReadyToCruise(injector_->vehicle_state(), frame,
                            GetContextAs<ParkAndGoContext>()->scenario_config);
  return FinishStage(ready_to_cruise);
}

StageResult ParkAndGoStageCheck::FinishStage(const bool success) {
  if (success) {
    next_stage_ = "PARK_AND_GO_CRUISE";
  } else {
    next_stage_ = "PARK_AND_GO_ADJUST";
  }
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_park_and_go()
      ->set_in_check_stage(false);
  return StageResult(StageStatusType::FINISHED);
}

void ParkAndGoStageCheck::ADCInitStatus() {
  auto* park_and_go_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_park_and_go();
  park_and_go_status->Clear();
  park_and_go_status->mutable_adc_init_position()->set_x(
      injector_->vehicle_state()->x());
  park_and_go_status->mutable_adc_init_position()->set_y(
      injector_->vehicle_state()->y());
  park_and_go_status->mutable_adc_init_position()->set_z(0.0);
  park_and_go_status->set_adc_init_heading(
      injector_->vehicle_state()->heading());
  park_and_go_status->set_in_check_stage(true);
}

}  // namespace planning
}  // namespace apollo
