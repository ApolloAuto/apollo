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

#include "modules/planning/scenarios/park/pull_over_emergency/pull_over_emergency_scenario.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace pull_over_emergency {

apollo::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
    PullOverEmergencyScenario::s_stage_factory_;

void PullOverEmergencyScenario::Init() {
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

void PullOverEmergencyScenario::RegisterStages() {
  if (!s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  /*
  s_stage_factory_.Register(
      ScenarioConfig::PULL_OVER_APPROACH,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new PullOverEmergencyStageApproach(config);
      });
      */
}

std::unique_ptr<Stage> PullOverEmergencyScenario::CreateStage(
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
bool PullOverEmergencyScenario::GetScenarioConfig() {
  if (!config_.has_pull_over_emergency_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(config_.pull_over_emergency_config());
  return true;
}

}  // namespace pull_over_emergency
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
