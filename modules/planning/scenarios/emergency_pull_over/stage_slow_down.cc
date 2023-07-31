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

#include "modules/planning/scenarios/emergency_pull_over/stage_slow_down.h"

#include <memory>

#include "cyber/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

StageResult EmergencyPullOverStageSlowDown::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: SlowDown";
  CHECK_NOTNULL(frame);

  auto scenario_context = GetContextAs<EmergencyPullOverContext>();
  scenario_config_.CopyFrom(scenario_context->scenario_config);

  // set cruise_speed to slow down
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  double target_slow_down_speed = scenario_context->target_slow_down_speed;
  if (target_slow_down_speed <= 0) {
    target_slow_down_speed = scenario_context->target_slow_down_speed =
        std::max(
            scenario_config_.target_slow_down_speed(),
            adc_speed - scenario_config_.max_stop_deceleration() *
                            scenario_config_.slow_down_deceleration_time());
  }
  auto& reference_line_info = frame->mutable_reference_line_info()->front();
  reference_line_info.LimitCruiseSpeed(target_slow_down_speed);

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "EmergencyPullOverStageSlowDown planning error";
  }

  // check slow enough
  static constexpr double kSpeedTolarence = 1.0;
  if (adc_speed - target_slow_down_speed <= kSpeedTolarence) {
    return FinishStage();
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult EmergencyPullOverStageSlowDown::FinishStage() {
  auto* pull_over_status = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_pull_over();
  pull_over_status->set_plan_pull_over_path(true);

  next_stage_ = "EMERGENCY_PULL_OVER_APPROACH";
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
