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
#include <utility>
#include <vector>

#include "modules/planning/proto/planning.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/scenarios/scenario.h"
#include "modules/planning/scenarios/stop_sign_unprotected/stop_sign_unprotected_scenario.h"  // NOINT

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign_protected {

struct StopSignUnprotectedContext;

DECLARE_STAGE(StopSignUnprotectedIntersectionCruise,
              StopSignUnprotectedContext);

class StopSignUnprotectedCreep : public Stage {
 public:
  explicit StopSignUnprotectedCreep(const ScenarioConfig::StageConfig& config)
      : Stage(config) {}
  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame);

 private:
  StopSignUnprotectedContext* GetContext() {
    return Stage::GetContextAs<StopSignUnprotectedContext>();
  }
};

class StopSignUnprotectedStop : public Stage {
 public:
  explicit StopSignUnprotectedStop(const ScenarioConfig::StageConfig& config)
      : Stage(config) {}
  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame);

 private:
  StopSignUnprotectedContext* GetContext() {
    return GetContextAs<StopSignUnprotectedContext>();
  }

  int RemoveWatchVehicle(
      const PathObstacle& path_obstacle,
      const std::vector<std::string>& watch_vehicle_ids,
      std::unordered_map<std::string, std::vector<std::string>>*
          watch_vehicles);
 private:
  const float conf_stop_duration_ = 1.0f;
  const double conf_min_pass_s_distance_ = 3.0;
};

class StopSignUnprotectedPreStop : public Stage {
 public:
  explicit StopSignUnprotectedPreStop(const ScenarioConfig::StageConfig& config)
      : Stage(config) {}
  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame);
 private:
  StopSignUnprotectedContext* GetContext() {
    return GetContextAs<StopSignUnprotectedContext>();
  }

  int AddWatchVehicle(const PathObstacle& path_obstacle,
                      std::unordered_map<std::string, std::vector<std::string>>*
                          watch_vehicles);
  bool CheckADCStop(const ReferenceLineInfo& reference_line_info);

 private:
  const double conf_watch_vehicle_max_valid_stop_distance_ = 5.0;
  const double conf_max_valid_stop_distance_ = 3.5;
  const double conf_max_adc_stop_speed_ = 0.3;
};

}  // namespace stop_sign_protected
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
