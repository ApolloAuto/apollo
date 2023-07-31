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

#include <string>
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/scenarios/valet_parking/stage_approaching_parking_spot.h"

namespace apollo {
namespace planning {
bool StageApproachingParkingSpot::Init(
    const StagePipeline& config,
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_dir, void* context) {
  if (!Stage::Init(config, injector, config_dir, context)) {
    return false;
  }
  scenario_config_.CopyFrom(
      GetContextAs<ValetParkingContext>()->scenario_config);
  return true;
}
StageResult StageApproachingParkingSpot::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: StageApproachingParkingSpot";
  CHECK_NOTNULL(frame);
  StageResult result;
  auto scenario_context = GetContextAs<ValetParkingContext>();

  if (scenario_context->target_parking_spot_id.empty()) {
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) =
      scenario_context->target_parking_spot_id;
  frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(
      scenario_context->pre_stop_rightaway_flag);
  *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point()) =
      scenario_context->pre_stop_rightaway_point;
  auto* reference_lines = frame->mutable_reference_line_info();
  for (auto& reference_line : *reference_lines) {
    auto* path_decision = reference_line.path_decision();
    if (nullptr == path_decision) {
      continue;
    }
    auto* dest_obstacle = path_decision->Find(FLAGS_destination_obstacle_id);
    if (nullptr == dest_obstacle) {
      continue;
    }
    ObjectDecisionType decision;
    decision.mutable_ignore();
    dest_obstacle->EraseDecision();
    dest_obstacle->AddLongitudinalDecision("ignore-dest-in-valet-parking",
                                           decision);
  }
  result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

  scenario_context->pre_stop_rightaway_flag =
      frame->open_space_info().pre_stop_rightaway_flag();
  scenario_context->pre_stop_rightaway_point =
      frame->open_space_info().pre_stop_rightaway_point();

  if (CheckADCStop(*frame)) {
    next_stage_ = "VALET_PARKING_PARKING";
    return StageResult(StageStatusType::FINISHED);
  }
  if (result.HasError()) {
    AERROR << "StopSignUnprotectedStagePreStop planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

bool StageApproachingParkingSpot::CheckADCStop(const Frame& frame) {
  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  if (adc_speed > max_adc_stop_speed) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;
  }

  // check stop close enough to stop line of the stop_sign
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double stop_fence_start_s =
      frame.open_space_info().open_space_pre_stop_fence_s();
  const double distance_stop_line_to_adc_front_edge =
      stop_fence_start_s - adc_front_edge_s;

  if (distance_stop_line_to_adc_front_edge >
      scenario_config_.max_valid_stop_distance()) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
