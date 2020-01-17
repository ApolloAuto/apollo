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

#include "modules/planning/scenarios/emergency/emergency_pull_over/stage_approach.h"

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
using apollo::common::VehicleConfigHelper;
using apollo::common::VehicleSignal;

EmergencyPullOverStageApproach::EmergencyPullOverStageApproach(
    const ScenarioConfig::StageConfig& config)
    : Stage(config) {}

Stage::StageStatus EmergencyPullOverStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  auto& reference_line_info = frame->mutable_reference_line_info()->front();

  // set vehicle signal
  reference_line_info.SetTurnSignal(VehicleSignal::TURN_RIGHT);

  double stop_line_s = 0.0;

  // add a stop fence
  const auto& pull_over_status =
      PlanningContext::Instance()->planning_status().pull_over();
  if (pull_over_status.has_position() && pull_over_status.position().has_x() &&
      pull_over_status.position().has_y()) {
    const auto& reference_line = reference_line_info.reference_line();
    common::SLPoint pull_over_sl;
    reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
    const double stop_distance = scenario_config_.stop_distance();
    stop_line_s =
        pull_over_sl.s() + stop_distance +
        VehicleConfigHelper::GetConfig().vehicle_param().front_edge_to_center();
    const std::string virtual_obstacle_id = "EMERGENCY_PULL_OVER";
    const std::vector<std::string> wait_for_obstacle_ids;
    planning::util::BuildStopDecision(
        virtual_obstacle_id, stop_line_s, stop_distance,
        StopReasonCode::STOP_REASON_PULL_OVER, wait_for_obstacle_ids,
        "EMERGENCY_PULL_OVER-scenario", frame,
        &(frame->mutable_reference_line_info()->front()));

    ADEBUG << "Build a stop fence for emergency_pull_over: id["
           << virtual_obstacle_id << "] s[" << stop_line_s << "]";
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "EmergencyPullOverStageApproach planning error";
  }

  if (stop_line_s > 0.0) {
    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    double distance = stop_line_s - adc_front_edge_s;
    const double adc_speed =
        common::VehicleStateProvider::Instance()->linear_velocity();
    const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                          ->GetConfig()
                                          .vehicle_param()
                                          .max_abs_speed_when_stopped();
    ADEBUG << "adc_speed[" << adc_speed << "] distance[" << distance << "]";
    static constexpr double kStopSpeedTolerance = 0.4;
    static constexpr double kStopDistanceTolerance = 3.0;
    if (adc_speed <= max_adc_stop_speed + kStopSpeedTolerance &&
        std::fabs(distance) <= kStopDistanceTolerance) {
      return FinishStage();
    }
  }

  return StageStatus::RUNNING;
}

Stage::StageStatus EmergencyPullOverStageApproach::FinishStage() {
  next_stage_ = ScenarioConfig::EMERGENCY_PULL_OVER_STANDBY;
  return Stage::FINISHED;
}

}  // namespace emergency_pull_over
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
