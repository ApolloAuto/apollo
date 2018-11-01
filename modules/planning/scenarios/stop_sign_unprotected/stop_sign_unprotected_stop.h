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

#include <string>
#include <unordered_map>
#include <vector>

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/scenarios/stage.h"
#include "modules/planning/scenarios/stop_sign_unprotected/stop_sign_unprotected_scenario.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign_protected {

struct StopSignUnprotectedContext;

class StopSignUnprotectedStop : public Stage {
 public:
  explicit StopSignUnprotectedStop(const ScenarioConfig::StageConfig& config)
      : Stage(config) {}

 private:
  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame) override;
  StopSignUnprotectedContext* GetContext() {
    return GetContextAs<StopSignUnprotectedContext>();
  }
  int RemoveWatchVehicle(
      const Obstacle& obstacle,
      const std::vector<std::string>& watch_vehicle_ids,
      std::unordered_map<std::string, std::vector<std::string>>*
          watch_vehicles);

 private:
  // TODO(all): move to scenario conf later
  const float conf_stop_duration_ = 1.0f;
  const double conf_min_pass_s_distance_ = 3.0;
  const float conf_wait_timeout_ = 8.0;
};

}  // namespace stop_sign_protected
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
