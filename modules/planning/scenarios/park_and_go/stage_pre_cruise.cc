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

#include "modules/planning/scenarios/park_and_go/stage_pre_cruise.h"

#include "cyber/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/scenarios/park_and_go/context.h"
#include "modules/planning/scenarios/park_and_go/util.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

StageResult ParkAndGoStagePreCruise::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Pre Cruise";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  const ScenarioParkAndGoConfig& scenario_config =
      GetContextAs<ParkAndGoContext>()->scenario_config;

  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  StageResult result = ExecuteTaskOnOpenSpace(frame);
  if (result.HasError()) {
    AERROR << "ParkAndGoStagePreCruise planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }
  // const bool ready_to_cruise =
  //     CheckADCReadyToCruise(frame, scenario_config_);
  auto vehicle_status = injector_->vehicle_state();
  AINFO << "Current steering percentage: "
        << vehicle_status->steering_percentage();;

  if ((std::fabs(vehicle_status->steering_percentage()) <
       scenario_config.max_steering_percentage_when_cruise()) &&
      CheckADCReadyToCruise(injector_->vehicle_state(), frame,
                            scenario_config)) {
    return FinishStage();
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult ParkAndGoStagePreCruise::FinishStage() {
  next_stage_ = "PARK_AND_GO_CRUISE";
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
