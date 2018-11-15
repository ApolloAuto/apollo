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

#ifndef MODULES_PLANNING_SCENARIOS_LANE_FOLLLOW_SCENARIO_H_
#define MODULES_PLANNING_SCENARIOS_LANE_FOLLLOW_SCENARIO_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/planning/scenarios/scenario.h"
#include "modules/planning/toolkits/optimizers/task.h"

namespace apollo {
namespace planning {

class LaneFollowScenario : public Scenario {
 public:
  LaneFollowScenario() : Scenario(ScenarioConfig::LANE_FOLLOW) {}
  virtual ~LaneFollowScenario() = default;

  bool Init() override;

  common::Status Process(const common::TrajectoryPoint& planning_init_point,
                         Frame* frame) override;

  ScenarioConfig::ScenarioType Transfer(
      const ScenarioConfig::ScenarioType& current_scenario,
      const common::TrajectoryPoint& ego_point,
      const Frame& frame) const override;

 private:
  void RegisterTasks();

  common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_start_point, Frame* frame,
      ReferenceLineInfo* reference_line_info);

  std::vector<common::SpeedPoint> DummyHotStart(
      const common::TrajectoryPoint& planning_init_point);

  void GenerateFallbackPathProfile(const ReferenceLineInfo* reference_line_info,
                                   PathData* path_data);

  common::SLPoint GetStopSL(const ObjectStop& stop_decision,
                            const ReferenceLine& reference_line) const;

  void RecordObstacleDebugInfo(ReferenceLineInfo* reference_line_info);

  void RecordDebugInfo(ReferenceLineInfo* reference_line_info,
                       const std::string& name, const double time_diff_ms);

  apollo::common::util::Factory<TaskType, Task> task_factory_;

  std::vector<std::unique_ptr<Task>> tasks_;

  ScenarioConfig config_;

  SpeedProfileGenerator speed_profile_generator_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_SCENARIOS_LANE_FOLLLOW_SCENARIO_H_
