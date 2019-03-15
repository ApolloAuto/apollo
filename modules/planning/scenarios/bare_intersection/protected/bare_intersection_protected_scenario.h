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
#include <string>

#include "modules/planning/proto/planning.pb.h"

#include "modules/common/util/factory.h"
#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace bare_intersection {

// stage context
struct BareIntersectionProtectedContext {
  ScenarioBareIntersectionProtectedConfig scenario_config;
};

class BareIntersectionProtectedScenario : public Scenario {
 public:
  explicit BareIntersectionProtectedScenario(
      const ScenarioConfig& config, const ScenarioContext* context)
      : Scenario(config, context) {}

  void Init() override;

  std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config);

  bool IsTransferable(const Scenario& current_scenario,
                      const Frame& frame) override;

  BareIntersectionProtectedContext* GetContext() { return &context_; }

 private:
  static void RegisterStages();
  bool GetScenarioConfig();
  static apollo::common::util::Factory<
      ScenarioConfig::StageType, Stage,
      Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
      s_stage_factory_;

 private:
  bool init_ = false;
  BareIntersectionProtectedContext context_;
};

}  // namespace bare_intersection
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
