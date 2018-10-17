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

namespace apollo {
namespace planning {

class StopSignUnprotectedScenario : public Scenario {
 public:
  // stage context
  struct StopSignUnprotectedContext {
    double stop_start_time;
    std::unordered_map<std::string, std::vector<std::string>> watch_vehicles;
    hdmap::PathOverlap next_stop_sign_overlap;
    std::vector<std::pair<hdmap::LaneInfoConstPtr, hdmap::OverlapInfoConstPtr>>
        associated_lanes;
  };

  StopSignUnprotectedScenario()
      : Scenario(FLAGS_scenario_stop_sign_unprotected_config_file) {}
  explicit StopSignUnprotectedScenario(const ScenarioConfig& config)
      : Scenario(config) {}

  void Observe(Frame* const frame);

  std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config);

  bool IsTransferable(const Scenario& current_scenario,
                      const common::TrajectoryPoint& ego_point,
                      const Frame& frame) const override;

  StopSignUnprotectedContext* GetContext() { return &context_; }

 private:
  static void RegisterStages();

  bool FindNextStopSign(const ReferenceLineInfo& reference_line_info);
  int GetAssociatedLanes(const hdmap::StopSignInfo& stop_sign_info);

 private:
  SpeedProfileGenerator speed_profile_generator_;

  StopSignUnprotectedContext context_;

  hdmap::StopSignInfoConstPtr next_stop_sign_ = nullptr;
  double adc_distance_to_stop_sign_;

  // TODO(all): move to scenario conf later
  const uint32_t conf_start_stop_sign_timer = 10;  // second

  static apollo::common::util::Factory<
      ScenarioConfig::StageType, Stage,
      Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
      s_stage_factory_;
};

DECLARE_STAGE(StopSignUnprotectedIntersectionCruise,
              StopSignUnprotectedScenario::StopSignUnprotectedContext);

class StopSignUnprotectedCreep : public Stage {
 public:
  explicit StopSignUnprotectedCreep(const ScenarioConfig::StageConfig& config)
      : Stage(config) {}
  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame);
  StopSignUnprotectedScenario::StopSignUnprotectedContext* GetContext() {
    return Stage::GetContextAs<
        StopSignUnprotectedScenario::StopSignUnprotectedContext>();
  }
};

class StopSignUnprotectedStop : public Stage {
 public:
  explicit StopSignUnprotectedStop(const ScenarioConfig::StageConfig& config)
      : Stage(config) {}
  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame);
  StopSignUnprotectedScenario::StopSignUnprotectedContext* GetContext() {
    return GetContextAs<
        StopSignUnprotectedScenario::StopSignUnprotectedContext>();
  }
  int RemoveWatchVehicle(
      const PathObstacle& path_obstacle,
      const std::vector<std::string>& watch_vehicle_ids,
      std::unordered_map<std::string, std::vector<std::string>>*
          watch_vehicles);
};

class StopSignUnprotectedPreStop : public Stage {
 public:
  explicit StopSignUnprotectedPreStop(const ScenarioConfig::StageConfig& config)
      : Stage(config) {}
  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame);
  int AddWatchVehicle(const PathObstacle& path_obstacle,
                      std::unordered_map<std::string, std::vector<std::string>>*
                          watch_vehicles);
  StopSignUnprotectedScenario::StopSignUnprotectedContext* GetContext() {
    return GetContextAs<
        StopSignUnprotectedScenario::StopSignUnprotectedContext>();
  }
  bool CheckADCStop(const ReferenceLineInfo& reference_line_info);
};

}  // namespace planning
}  // namespace apollo
