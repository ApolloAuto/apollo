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

#include "modules/common/util/factory.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign {

// stage context
struct StopSignUnprotectedContext {
  ScenarioStopSignUnprotectedConfig scenario_config;
  std::string stop_sign_id;
  double stop_start_time = 0.0;
  double creep_start_time = 0.0;
  std::unordered_map<std::string, std::vector<std::string>> watch_vehicles;
  std::vector<std::pair<hdmap::LaneInfoConstPtr, hdmap::OverlapInfoConstPtr>>
      associated_lanes;
};

class StopSignUnprotectedScenario : public Scenario {
 public:
  explicit StopSignUnprotectedScenario(const ScenarioConfig& config,
                                       const ScenarioContext* context)
      : Scenario(config, context) {}

  void Init() override;

  std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config);

  bool IsTransferable(const Scenario& current_scenario,
                      const common::TrajectoryPoint& ego_point,
                      const Frame& frame) override;

  StopSignUnprotectedContext* GetContext() { return &context_; }

 private:
  static void RegisterStages();
  bool GetScenarioConfig();
  int GetAssociatedLanes(const hdmap::StopSignInfo& stop_sign_info);

 private:
  bool init_ = false;
  StopSignUnprotectedContext context_;

  hdmap::StopSignInfoConstPtr stop_sign_ = nullptr;

  static apollo::common::util::Factory<
      ScenarioConfig::StageType, Stage,
      Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
      s_stage_factory_;
};

}  // namespace stop_sign
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
