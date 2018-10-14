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

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/planning/scenarios/scenario.h"
#include "modules/planning/toolkits/task.h"

namespace apollo {
namespace planning {

DECLARE_STAGE(SidePassApproachObstacle);
DECLARE_STAGE(SidePassGeneratePath);
DECLARE_STAGE(SidePassStopOnWaitPoint);
DECLARE_STAGE(SidePassDetectSafety);
DECLARE_STAGE(SidePassPassObstacle);

class SidePassScenario : public Scenario {
 public:
  SidePassScenario() : Scenario(FLAGS_scenario_side_pass_config_file) {}
  explicit SidePassScenario(const ScenarioConfig& config) : Scenario(config) {}

  bool IsTransferable(const Scenario& current_scenario,
                      const common::TrajectoryPoint& ego_point,
                      const Frame& frame) const override;

  std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config) const;

 private:
  static void RegisterStages();

  bool IsSidePassScenario(const common::TrajectoryPoint& planning_start_point,
                          const Frame& frame) const;

  bool IsFarFromIntersection(const Frame& frame);

  bool HasBlockingObstacle(const SLBoundary& adc_sl_boundary,
                           const PathDecision& path_decision) const;

 private:
  static apollo::common::util::Factory<
      ScenarioConfig::StageType, Stage,
      Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
      s_stage_factory_;

  std::vector<std::unique_ptr<Task>> tasks_;
  ScenarioConfig config_;
  bool stage_init_ = false;
  SpeedProfileGenerator speed_profile_generator_;
  PathData path_;
  double wait_point_s = 0;
  PathData path_data_;
};

}  // namespace planning
}  // namespace apollo
