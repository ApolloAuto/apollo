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

#include "modules/planning/scenarios/park/pull_over/pull_over_scenario.h"
#include "modules/planning/scenarios/stage.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace pull_over {

struct PullOverContext;

class PullOverStageRetryApproachParking : public Stage {
 public:
  PullOverStageRetryApproachParking(
      const ScenarioConfig::StageConfig& config,
      const std::shared_ptr<DependencyInjector>& injector);

  StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

  PullOverContext* GetContext() {
    return Stage::GetContextAs<PullOverContext>();
  }

  Stage::StageStatus FinishStage();

  bool CheckADCStop(const Frame& frame);

 private:
  ScenarioPullOverConfig scenario_config_;
};
}  // namespace pull_over
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
