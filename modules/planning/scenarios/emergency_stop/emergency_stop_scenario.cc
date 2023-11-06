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

#include "modules/planning/scenarios/emergency_stop/emergency_stop_scenario.h"

#include "cyber/common/log.h"
#include "modules/planning/scenarios/emergency_stop/stage_approach.h"
#include "modules/planning/scenarios/emergency_stop/stage_standby.h"

namespace apollo {
namespace planning {

bool EmergencyStopScenario::Init(std::shared_ptr<DependencyInjector> injector,
                                 const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<apollo::planning::ScenarioEmergencyStopConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get config of scenario" << Name();
    return false;
  }

  init_ = true;
  return true;
}

bool EmergencyStopScenario::IsTransferable(const Scenario* const other_scenario,
                                           const Frame& frame) {
  const auto& pad_msg_driving_action = frame.GetPadMsgDrivingAction();
  if (pad_msg_driving_action == PadMessage::STOP) {
    return true;
  }
  return false;
}

ScenarioResult EmergencyStopScenario::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ScenarioResult stage_result;
  if (frame->reference_line_info().empty()) {
    stage_result.SetStageResult(StageStatusType::ERROR,
                                "Reference line is empty!");
    AERROR << "Reference line is empty in EmergencyStopScenario!";
    return stage_result;
  }
  return Scenario::Process(planning_init_point, frame);
}

bool EmergencyStopScenario::Exit(Frame* frame) {
  auto* emergency_stop = injector_->planning_context()
                             ->mutable_planning_status()
                             ->mutable_emergency_stop();
  emergency_stop->Clear();
  return true;
}

}  // namespace planning
}  // namespace apollo
