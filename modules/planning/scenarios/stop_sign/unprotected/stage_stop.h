/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include <unordered_map>
#include <vector>

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/scenarios/stage.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stop_sign_unprotected_scenario.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign {

struct StopSignUnprotectedContext;

class StopSignUnprotectedStageStop : public Stage {
 public:
  StopSignUnprotectedStageStop(
      const ScenarioConfig::StageConfig& config,
      const std::shared_ptr<DependencyInjector>& injector)
      : Stage(config, injector) {}

 private:
  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame) override;
  StopSignUnprotectedContext* GetContext() {
    return GetContextAs<StopSignUnprotectedContext>();
  }
  int RemoveWatchVehicle(
      const PathDecision& path_decision,
      std::unordered_map<std::string, std::vector<std::string>>*
          watch_vehicles);

 private:
  Stage::StageStatus FinishStage();

 private:
  ScenarioStopSignUnprotectedConfig scenario_config_;
};

}  // namespace stop_sign
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
