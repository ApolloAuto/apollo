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

#include "modules/planning/scenarios/park/pull_over/stage_approach.h"

#include "cyber/common/log.h"

#include "modules/planning/common/frame.h"
#include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace pull_over {

using common::TrajectoryPoint;

PullOverStageApproach::PullOverStageApproach(
    const ScenarioConfig::StageConfig& config)
    : Stage(config) {}

Stage::StageStatus PullOverStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "PullOverStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  scenario::util::PullOverStatus status =
      scenario::util::CheckADCPullOver(reference_line_info, scenario_config_);

  if (status == scenario::util::UNKNOWN ||
      status == scenario::util::PASS_DESTINATION ||
      status == scenario::util::PARK_COMPLETE) {
    return FinishStage(true);
  } else if (status == scenario::util::PARK_FAIL) {
    return FinishStage(false);
  }

  return StageStatus::RUNNING;
}

Stage::StageStatus PullOverStageApproach::FinishStage(const bool success) {
  if (success) {
    return FinishScenario();
  } else {
    next_stage_ = ScenarioConfig::PULL_OVER_RETRY_APPROACH_PARKING;
    return Stage::FINISHED;
  }
}

}  // namespace pull_over
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
