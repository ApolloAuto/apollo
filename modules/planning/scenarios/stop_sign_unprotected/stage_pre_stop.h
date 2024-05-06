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
 * @file stage_pre_stop.h
 **/

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/scenario_base/stage.h"

namespace apollo {
namespace planning {

class StopSignUnprotectedStagePreStop : public Stage {
 public:
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

 private:
  int AddWatchVehicle(const Obstacle& obstacle,
                      std::unordered_map<std::string, std::vector<std::string>>*
                          watch_vehicles);

  bool CheckADCStop(const double adc_front_edge_s, const double stop_line_s);

  StageResult FinishStage();
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::StopSignUnprotectedStagePreStop, Stage)

}  // namespace planning
}  // namespace apollo
