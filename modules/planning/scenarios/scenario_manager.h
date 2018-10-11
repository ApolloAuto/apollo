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

#pragma once

#include <memory>
#include <set>

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {

class ScenarioManager final {
 public:
  ScenarioManager() = default;

  bool Init(const std::set<ScenarioConfig::ScenarioType>& supported_scenarios);

  Scenario* mutable_scenario() { return current_scenario_.get(); }

  void Update(const common::TrajectoryPoint& ego_point, const Frame& frame);

 private:
  bool SelectChangeLaneScenario(const common::TrajectoryPoint& ego_point,
                                const Frame& frame);
  bool ReuseCurrentScenario(const common::TrajectoryPoint& ego_point,
                            const Frame& frame);
  bool SelectScenario(const ScenarioConfig::ScenarioType type,
                      const common::TrajectoryPoint& ego_point,
                      const Frame& frame);

  void RegisterScenarios();

  ScenarioConfig::ScenarioType DecideCurrentScenario(
      const common::TrajectoryPoint& ego_point, const Frame& frame);

  common::util::Factory<ScenarioConfig::ScenarioType, Scenario>
      scenario_factory_;

  std::unique_ptr<Scenario> current_scenario_;
  ScenarioConfig::ScenarioType default_scenario_type_;
  std::set<ScenarioConfig::ScenarioType> supported_scenarios_;
};

}  // namespace planning
}  // namespace apollo
