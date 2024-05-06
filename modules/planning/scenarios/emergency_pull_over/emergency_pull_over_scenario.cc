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

#include "modules/planning/scenarios/emergency_pull_over/emergency_pull_over_scenario.h"

#include <memory>

#include "cyber/common/log.h"
#include "modules/planning/scenarios/emergency_pull_over/stage_approach.h"
#include "modules/planning/scenarios/emergency_pull_over/stage_slow_down.h"
#include "modules/planning/scenarios/emergency_pull_over/stage_standby.h"

namespace apollo {
namespace planning {

bool EmergencyPullOverScenario::Init(
    std::shared_ptr<DependencyInjector> injector, const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioEmergencyPullOverConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get config of scenario" << Name();
    return false;
  }

  init_ = true;
  return true;
}

bool EmergencyPullOverScenario::IsTransferable(
    const Scenario* const other_scenario, const Frame& frame) {
  if (frame.reference_line_info().empty()) {
    return false;
  }
  const auto& pad_msg_driving_action = frame.GetPadMsgDrivingAction();
  if (pad_msg_driving_action == PadMessage::PULL_OVER) {
    return true;
  }
  return false;
}

bool EmergencyPullOverScenario::Exit(Frame* frame) {
  injector_->planning_context()->mutable_planning_status()->clear_pull_over();
  return true;
}

bool EmergencyPullOverScenario::Enter(Frame* frame) {
  auto* pull_over = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_pull_over();
  pull_over->set_pull_over_type(PullOverStatus::EMERGENCY_PULL_OVER);
  return true;
}

}  // namespace planning
}  // namespace apollo
