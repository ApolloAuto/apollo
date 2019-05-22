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
#include <string>

#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/map/proto/map_id.pb.h"
#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace valet_parking {

struct ValetParkingContext {
  ScenarioValetParkingConfig scenario_config;
  std::string target_parking_spot_id;
  bool pre_stop_rightaway_flag = false;
  hdmap::MapPathPoint pre_stop_rightaway_point;
};

class ValetParkingScenario : public Scenario {
 public:
  ValetParkingScenario(const ScenarioConfig& config,
                       const ScenarioContext* context)
      : Scenario(config, context) {}

  void Init() override;

  std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config) override;

  static bool IsTransferable(const Frame& frame,
                             const double parking_start_range);

  ValetParkingContext* GetContext() { return &context_; }

 private:
  static void RegisterStages();
  bool GetScenarioConfig();
  static bool SearchTargetParkingSpotOnPath(
      const hdmap::Path& nearby_path, const std::string& target_parking_id,
      hdmap::PathOverlap* parking_space_overlap);
  static bool CheckDistanceToParkingSpot(
      const common::VehicleState& vehicle_state, const hdmap::Path& nearby_path,
      const double parking_start_range,
      const hdmap::PathOverlap& parking_space_overlap);

 private:
  bool init_ = false;
  static apollo::common::util::Factory<
      ScenarioConfig::StageType, Stage,
      Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
      s_stage_factory_;
  ValetParkingContext context_;
  const hdmap::HDMap* hdmap_ = nullptr;
};

}  // namespace valet_parking
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
