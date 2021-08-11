/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/dead_end/deadend_turnaround/deadend_turnaround_scenario.h"
#include "modules/planning/scenarios/stage.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace deadend_turnaround {

class StageTurning : public Stage {
 public:
  StageTurning(const ScenarioConfig::StageConfig& config,
               const std::shared_ptr<DependencyInjector>& injector)
      : Stage(config, injector) {}

 private:
  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame) override;

  DeadEndTurnAroundContext* GetContext() {
    return GetContextAs<DeadEndTurnAroundContext>();
  }

 private:
  Stage::StageStatus FinishStage();

 private:
  ScenarioDeadEndTurnAroundConfig scenario_config_;
};

}  // namespace deadend_turnaround
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
