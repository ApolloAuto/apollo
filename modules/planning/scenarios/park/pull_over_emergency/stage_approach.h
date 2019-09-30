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

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/scenarios/park/pull_over_emergency/pull_over_emergency_scenario.h"
#include "modules/planning/scenarios/stage.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace pull_over_emergency {

struct PullOverEmergencyContext;

class PullOverEmergencyStageApproach : public Stage {
 public:
  explicit PullOverEmergencyStageApproach(
      const ScenarioConfig::StageConfig& config);

  StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

  PullOverEmergencyContext* GetContext() {
    return Stage::GetContextAs<PullOverEmergencyContext>();
  }

  Stage::StageStatus FinishStage(const bool success);

 private:
  ScenarioPullOverEmergencyConfig scenario_config_;
};

}  // namespace pull_over_emergency
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
