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

#include "modules/planning/scenarios/emergency/emergency_pull_over/stage_slow_down.h"

#include <string>
#include <vector>

#include "cyber/common/log.h"

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace emergency_pull_over {

using apollo::common::TrajectoryPoint;

EmergencyPullOverStageSlowDown::EmergencyPullOverStageSlowDown(
    const ScenarioConfig::StageConfig& config)
    : Stage(config) {}

Stage::StageStatus EmergencyPullOverStageSlowDown::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: SlowDown";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  // set cruise_speed to slow down
  const double adc_speed =
      common::VehicleStateProvider::Instance()->linear_velocity();
  const double target_speed =
      adc_speed - scenario_config_.max_stop_deceleration() *
                      scenario_config_.slow_down_deceleration_time();
  auto& reference_line_info = frame->mutable_reference_line_info()->front();
  reference_line_info.SetCruiseSpeed(target_speed);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "EmergencyPullOverStageSlowDown planning error";
  }

  // check slow enough
  constexpr double kSpeedTolarence = 1.0;
  if (adc_speed - target_speed <= kSpeedTolarence) {
    return FinishStage();
  }

  return StageStatus::RUNNING;
}

Stage::StageStatus EmergencyPullOverStageSlowDown::FinishStage() {
  next_stage_ = ScenarioConfig::EMERGENCY_PULL_OVER_APPROACH;
  return Stage::FINISHED;
}

}  // namespace emergency_pull_over
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
