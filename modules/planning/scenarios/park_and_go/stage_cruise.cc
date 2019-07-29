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

#include "modules/planning/scenarios/park_and_go/stage_cruise.h"

#include <string>
#include <vector>

#include "cyber/common/log.h"

#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace park_and_go {

using common::TrajectoryPoint;

Stage::StageStatus ParkAndGoStageCruise::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Cruise";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  // cruise w/o refernce line
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  bool plan_ok = ExecuteTaskOnOpenSpace(frame);
  if (!plan_ok) {
    AERROR << "ParkAndGoStageCruise planning error";
    return StageStatus::ERROR;
  }

  const ReferenceLineInfo& reference_line_info =
      frame->reference_line_info().front();
  // check ADC status:
  // 1. At routing begining: stage finished
  scenario::util::ParkAndGoStatus status =
      scenario::util::CheckADCParkAndGoOpenSpace(reference_line_info,
                                                 scenario_config_);

  // reach the beginning of reference line
  if ((status == scenario::util::CRUISE_COMPLETE)) {
    return FinishStage();
  }
  return Stage::RUNNING;
}

Stage::StageStatus ParkAndGoStageCruise::FinishStage() {
  return FinishScenario();
}

}  // namespace park_and_go
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
