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

#include "modules/planning/scenarios/park/pull_over/pull_over_scenario.h"

#include "cyber/common/log.h"
#include "modules/planning/scenarios/park/pull_over/stage_approach.h"
#include "modules/planning/scenarios/park/pull_over/stage_retry_approach_parking.h"
#include "modules/planning/scenarios/park/pull_over/stage_retry_parking.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace pull_over {

apollo::common::util::Factory<
    StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config,
               const std::shared_ptr<DependencyInjector>& injector)>
    PullOverScenario::s_stage_factory_;

void PullOverScenario::Init() {
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

void PullOverScenario::RegisterStages() {
  if (!s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      StageType::PULL_OVER_APPROACH,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new PullOverStageApproach(config, injector);
      });
  s_stage_factory_.Register(
      StageType::PULL_OVER_RETRY_APPROACH_PARKING,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new PullOverStageRetryApproachParking(config, injector);
      });
  s_stage_factory_.Register(
      StageType::PULL_OVER_RETRY_PARKING,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new PullOverStageRetryParking(config, injector);
      });
}

std::unique_ptr<Stage> PullOverScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (s_stage_factory_.Empty()) {
    RegisterStages();
  }
  auto ptr = s_stage_factory_.CreateObjectOrNull(stage_config.stage_type(),
                                                 stage_config, injector);
  if (ptr) {
    ptr->SetContext(&context_);
  }
  return ptr;
}

/*
 * read scenario specific configs and set in context_ for stages to read
 */
bool PullOverScenario::GetScenarioConfig() {
  if (!config_.has_pull_over_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(config_.pull_over_config());
  return true;
}

}  // namespace pull_over
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
