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
#include <vector>

#include "modules/common/util/factory.h"
#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

struct SidePassContext {
  PathData path_data_;
  ScenarioSidePassConfig scenario_config_;
  int pass_obstacle_stuck_cycle_num_;
  uint32_t backup_stage_cycle_num_;
  std::string front_blocking_obstacle_id_;
};

class SidePassScenario : public Scenario {
 public:
  explicit SidePassScenario(const ScenarioConfig& config,
                            const ScenarioContext* scenario_context);

  bool IsTransferable(const Scenario& current_scenario,
                      const common::TrajectoryPoint& ego_point,
                      const Frame& frame) override;

  std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config) override;

 private:
  static void RegisterStages();

  bool IsSidePassScenario(const Frame& frame);

  bool IsFarFromIntersection(const Frame& frame);

  bool IsFarFromDestination(const Frame& frame);

  bool HasBlockingObstacle(const Frame& frame);

  bool IsParked(const ReferenceLine& reference_line, const Obstacle* obstacle);

 private:
  static apollo::common::util::Factory<
      ScenarioConfig::StageType, Stage,
      Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
      s_stage_factory_;
  SidePassContext side_pass_context_;
  std::vector<std::unique_ptr<Task>> tasks_;
  ScenarioConfig config_;
  bool stage_init_ = false;
  std::string front_blocking_obstacle_id_;
};

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
