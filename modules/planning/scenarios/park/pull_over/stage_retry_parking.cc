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

#include "modules/planning/scenarios/park/pull_over/stage_retry_parking.h"

#include "cyber/common/log.h"

#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace pull_over {

using common::TrajectoryPoint;

PullOverStageRetryParking::PullOverStageRetryParking(
    const ScenarioConfig::StageConfig& config)
    : Stage(config) {}

Stage::StageStatus PullOverStageRetryParking::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: RetryParking";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  // Open space planning doesn't use planning_init_point from upstream because
  // of different stitching strategy
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  bool plan_ok = ExecuteTaskOnOpenSpace(frame);
  if (!plan_ok) {
    AERROR << "PullOverStageRetryParking planning error";
    return StageStatus::ERROR;
  }

  *(frame->mutable_open_space_info()
        ->mutable_debug()
        ->mutable_planning_data()
        ->mutable_pull_over_status()) =
      PlanningContext::Instance()->planning_status().pull_over();
  frame->mutable_open_space_info()->sync_debug_instance();

  scenario::util::PullOverStatus status =
      scenario::util::CheckADCPullOverOpenSpace(scenario_config_);
  if ((status == scenario::util::PASS_DESTINATION ||
       status == scenario::util::PARK_COMPLETE) &&
      FLAGS_enable_pull_over_exit) {
    return FinishStage();
  }
  return StageStatus::RUNNING;
}

Stage::StageStatus PullOverStageRetryParking::FinishStage() {
  return FinishScenario();
}

}  // namespace pull_over
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
