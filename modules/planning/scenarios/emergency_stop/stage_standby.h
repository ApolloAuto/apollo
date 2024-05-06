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

#include "modules/planning/planning_base/proto/planning_config.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/scenario_base/stage.h"
#include "modules/planning/scenarios/emergency_stop/emergency_stop_scenario.h"

namespace apollo {
namespace planning {

struct EmergencyStopContext;

class EmergencyStopStageStandby : public Stage {
 public:
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

 private:
  StageResult FinishStage();

  ScenarioEmergencyStopConfig scenario_config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::EmergencyStopStageStandby, Stage)

}  // namespace planning
}  // namespace apollo
