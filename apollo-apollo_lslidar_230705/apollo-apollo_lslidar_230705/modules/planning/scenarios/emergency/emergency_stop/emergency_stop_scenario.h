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

#pragma once

#include <memory>

#include "modules/common/util/factory.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace emergency_stop {

// stage context
struct EmergencyStopContext {
  ScenarioEmergencyStopConfig scenario_config;
};

class EmergencyStopScenario : public Scenario {
 public:
  EmergencyStopScenario(const ScenarioConfig& config,
                        const ScenarioContext* context,
                        const std::shared_ptr<DependencyInjector>& injector)
      : Scenario(config, context, injector) {}

  void Init() override;

  std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config,
      const std::shared_ptr<DependencyInjector>& injector);

  EmergencyStopContext* GetContext() { return &context_; }

 private:
  static void RegisterStages();
  bool GetScenarioConfig();

 private:
  static apollo::common::util::Factory<
      StageType, Stage,
      Stage* (*)(const ScenarioConfig::StageConfig& stage_config,
                 const std::shared_ptr<DependencyInjector>& injector)>
      s_stage_factory_;
  bool init_ = false;
  EmergencyStopContext context_;
};

}  // namespace emergency_stop
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
