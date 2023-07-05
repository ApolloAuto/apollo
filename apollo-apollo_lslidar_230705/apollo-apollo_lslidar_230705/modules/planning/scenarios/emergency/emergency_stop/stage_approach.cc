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

#include "modules/planning/scenarios/emergency/emergency_stop/stage_approach.h"

#include <memory>
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
namespace emergency_stop {

using apollo::common::TrajectoryPoint;

EmergencyStopStageApproach::EmergencyStopStageApproach(
    const ScenarioConfig::StageConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Stage(config, injector) {}

Stage::StageStatus EmergencyStopStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  // set vehicle signal
  frame->mutable_reference_line_info()->front().SetEmergencyLight();

  // add a stop fence
  const auto& reference_line_info = frame->reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double stop_distance = scenario_config_.stop_distance();

  bool stop_fence_exist = false;
  double stop_line_s;
  const auto& emergency_stop_status =
      injector_->planning_context()->planning_status().emergency_stop();
  if (emergency_stop_status.has_stop_fence_point()) {
    common::SLPoint stop_fence_sl;
    reference_line.XYToSL(emergency_stop_status.stop_fence_point(),
                          &stop_fence_sl);
    if (stop_fence_sl.s() > adc_front_edge_s) {
      stop_fence_exist = true;
      stop_line_s = stop_fence_sl.s();
    }
  }

  if (!stop_fence_exist) {
    const double deceleration = scenario_config_.max_stop_deceleration();
    const double travel_distance =
        std::ceil(std::pow(adc_speed, 2) / (2 * deceleration));

    static constexpr double kBuffer = 2.0;
    stop_line_s = adc_front_edge_s + travel_distance + stop_distance + kBuffer;
    ADEBUG << "travel_distance[" << travel_distance << "] [" << adc_speed
           << "] adc_front_edge_s[" << adc_front_edge_s << "] stop_line_s["
           << stop_line_s << "]";
    const auto& stop_fence_point =
        reference_line.GetReferencePoint(stop_line_s);
    auto* emergency_stop_fence_point = injector_->planning_context()
                                           ->mutable_planning_status()
                                           ->mutable_emergency_stop()
                                           ->mutable_stop_fence_point();
    emergency_stop_fence_point->set_x(stop_fence_point.x());
    emergency_stop_fence_point->set_y(stop_fence_point.y());
  }

  const std::string virtual_obstacle_id = "EMERGENCY_STOP";
  const std::vector<std::string> wait_for_obstacle_ids;
  planning::util::BuildStopDecision(
      virtual_obstacle_id, stop_line_s, stop_distance,
      StopReasonCode::STOP_REASON_EMERGENCY, wait_for_obstacle_ids,
      "EMERGENCY_STOP-scenario", frame,
      &(frame->mutable_reference_line_info()->front()));
  ADEBUG << "Build a stop fence for emergency_stop: id[" << virtual_obstacle_id
         << "] s[" << stop_line_s << "]";

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "EmergencyPullOverStageApproach planning error";
  }

  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  if (adc_speed <= max_adc_stop_speed) {
    return FinishStage();
  }

  return StageStatus::RUNNING;
}

Stage::StageStatus EmergencyStopStageApproach::FinishStage() {
  next_stage_ = StageType::EMERGENCY_STOP_STANDBY;
  return Stage::FINISHED;
}

}  // namespace emergency_stop
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
