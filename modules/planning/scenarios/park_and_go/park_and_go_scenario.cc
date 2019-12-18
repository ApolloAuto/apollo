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

#include "modules/planning/scenarios/park_and_go/park_and_go_scenario.h"

#include "cyber/common/log.h"
#include "modules/planning/scenarios/park_and_go/stage_adjust.h"
#include "modules/planning/scenarios/park_and_go/stage_check.h"
#include "modules/planning/scenarios/park_and_go/stage_cruise.h"
#include "modules/planning/scenarios/park_and_go/stage_pre_cruise.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace park_and_go {

apollo::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
    ParkAndGoScenario::s_stage_factory_;

void ParkAndGoScenario::Init() {
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

void ParkAndGoScenario::RegisterStages() {
  if (!s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      ScenarioConfig::PARK_AND_GO_CHECK,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new ParkAndGoStageCheck(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::PARK_AND_GO_ADJUST,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new ParkAndGoStageAdjust(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::PARK_AND_GO_PRE_CRUISE,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new ParkAndGoStagePreCruise(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::PARK_AND_GO_CRUISE,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new ParkAndGoStageCruise(config);
      });
}

std::unique_ptr<Stage> ParkAndGoScenario::CreateStage(
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

bool ParkAndGoScenario::GetScenarioConfig() {
  if (!config_.has_park_and_go_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(config_.park_and_go_config());
  return true;
}

}  // namespace park_and_go
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
