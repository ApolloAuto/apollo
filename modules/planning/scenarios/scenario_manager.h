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

#ifndef MODULES_PLANNING_SCENARIOS_SCENARIO_MANAGER_H_
#define MODULES_PLANNING_SCENARIOS_SCENARIO_MANAGER_H_

#include <memory>

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {

class ScenarioManager final {
 public:
  ScenarioManager() = default;

  bool Init();

  Scenario* mutable_scenario() { return scenario_.get(); }

  void Update(const common::TrajectoryPoint& ego_point, const Frame& frame);

 private:
  void RegisterScenarios();

  ScenarioConfig::ScenarioType DecideCurrentScenario(
      const common::TrajectoryPoint& ego_point, const Frame& frame);

  common::util::Factory<ScenarioConfig::ScenarioType, Scenario>
      scenario_factory_;

  std::unique_ptr<Scenario> scenario_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_SCENARIOS_SCENARIO_MANAGER_H_
