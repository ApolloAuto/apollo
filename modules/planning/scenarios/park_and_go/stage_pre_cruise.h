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

#pragma once

#include <memory>

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/scenarios/park_and_go/park_and_go_scenario.h"
#include "modules/planning/scenarios/stage.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace park_and_go {

struct ParkAndGoContext;

class ParkAndGoStagePreCruise : public Stage {
 public:
  ParkAndGoStagePreCruise(const ScenarioConfig::StageConfig& config,
                          const std::shared_ptr<DependencyInjector>& injector)
      : Stage(config, injector) {}

  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame) override;

  ParkAndGoContext* GetContext() {
    return Stage::GetContextAs<ParkAndGoContext>();
  }

  Stage::StageStatus FinishStage();

 private:
  ScenarioParkAndGoConfig scenario_config_;
};

}  // namespace park_and_go
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
