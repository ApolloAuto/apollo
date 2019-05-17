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

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace pull_over {

using common::TrajectoryPoint;
using common::util::DistanceXY;

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
  PullOverStatus status = CheckADCStop(reference_line_info);
  if (status == PASS || status == PARK) {
    return FinishStage(true);
  } else if (status == STUCK) {
    return FinishStage(false);
  }

  return StageStatus::RUNNING;
}

/**
 * @brief: check adc parked properly
 */
PullOverStageApproach::PullOverStatus PullOverStageApproach::CheckADCStop(
      const ReferenceLineInfo& reference_line_info) {
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const auto& pull_over_status =
      PlanningContext::Instance()->planning_status().pull_over();
  double distance = adc_front_edge_s - pull_over_status.pull_over_s();
  constexpr double kPassDistance = 8.0;  // meter
  if (distance >= kPassDistance) {
    ADEBUG << "ADC passed pull-over spot: distance[" << distance << "]";
    return PASS;
  }

  const double adc_speed =
      common::VehicleStateProvider::Instance()->linear_velocity();
  if (adc_speed > scenario_config_.max_adc_stop_speed()) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return APPOACHING;
  }

  constexpr double kParkCheckRange = 3.0;  // meter
  if (distance <= -kParkCheckRange) {
    ADEBUG << "ADC still far: distance[" << distance << "]";
    return APPOACHING;
  }

  constexpr double kPositionTolerance = 0.3;  // meter
  common::math::Vec2d adc_position = {
      common::VehicleStateProvider::Instance()->x(),
      common::VehicleStateProvider::Instance()->y()};
  common::math::Vec2d pull_over_position = {
      pull_over_status.pull_over_x(),
      pull_over_status.pull_over_y()};
  distance = DistanceXY(adc_position, pull_over_position);
  ADEBUG << "adc_position(" << adc_position.x() << ", " << adc_position.y()
         << ") pull_over_position(" << pull_over_position.x()
         << ", " << pull_over_position.y()
         << ") distance[" << distance << "]";

  return distance <= kPositionTolerance ? PARK : STUCK;
}

Stage::StageStatus PullOverStageApproach::FinishStage(const bool success) {
  if (success) {
    return FinishScenario();
  } else {
    next_stage_ = ScenarioConfig::PULL_OVER_RETRY_PARKING;
    return Stage::FINISHED;
  }
}

}  // namespace pull_over
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
