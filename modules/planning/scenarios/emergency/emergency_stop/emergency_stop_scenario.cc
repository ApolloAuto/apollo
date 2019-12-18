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

#include "modules/planning/scenarios/emergency/emergency_stop/emergency_stop_scenario.h"

#include "cyber/common/log.h"
#include "modules/planning/scenarios/emergency/emergency_stop/stage_approach.h"
#include "modules/planning/scenarios/emergency/emergency_stop/stage_standby.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace emergency_stop {

apollo::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
    EmergencyStopScenario::s_stage_factory_;

void EmergencyStopScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();

  if (!GetScenarioConfig()) {
    AERROR << "fail to get scenario specific config";
    return;
  }

  init_ = true;
}

void EmergencyStopScenario::RegisterStages() {
  if (!s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      ScenarioConfig::EMERGENCY_STOP_APPROACH,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new EmergencyStopStageApproach(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::EMERGENCY_STOP_STANDBY,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new EmergencyStopStageStandby(config);
      });
}

std::unique_ptr<Stage> EmergencyStopScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config) {
  if (s_stage_factory_.Empty()) {
    RegisterStages();
  }
  auto ptr = s_stage_factory_.CreateObjectOrNull(stage_config.stage_type(),
                                                 stage_config);
  if (ptr) {
    ptr->SetContext(&context_);
  }
  return ptr;
}

/*
 * read scenario specific configs and set in context_ for stages to read
 */
bool EmergencyStopScenario::GetScenarioConfig() {
  if (!config_.has_emergency_stop_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(config_.emergency_stop_config());
  return true;
}

}  // namespace emergency_stop
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
