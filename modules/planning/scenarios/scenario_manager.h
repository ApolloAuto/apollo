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
#include <unordered_map>

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {
namespace scenario {

class ScenarioManager final {
 public:
  ScenarioManager() = default;

  bool Init(const std::set<ScenarioConfig::ScenarioType>& supported_scenarios);

  Scenario* mutable_scenario() { return current_scenario_.get(); }

  void Update(const common::TrajectoryPoint& ego_point, const Frame& frame);

 private:
  void Observe(const Frame& frame);

  std::unique_ptr<Scenario> CreateScenario(
      ScenarioConfig::ScenarioType scenario_type);

  void RegisterScenarios();

  ScenarioConfig::ScenarioType SelectBareIntersectionScenario(
      const Frame& frame, const hdmap::PathOverlap& pnc_junction_overlap);

  ScenarioConfig::ScenarioType SelectChangeLaneScenario(const Frame& frame);

  ScenarioConfig::ScenarioType SelectPullOverScenario(const Frame& frame);

  ScenarioConfig::ScenarioType SelectStopSignScenario(
      const Frame& frame, const hdmap::PathOverlap& stop_sign_overlap);

  ScenarioConfig::ScenarioType SelectTrafficLightScenario(
      const Frame& frame, const hdmap::PathOverlap& traffic_light_overlap);

  ScenarioConfig::ScenarioType SelectValetParkingScenario(const Frame& frame);

  ScenarioConfig::ScenarioType SelectYieldSignScenario(
      const Frame& frame, const hdmap::PathOverlap& yield_sign_overlap);

  // functions for scenario voter implementation
  // do NOT delete the code yet
  // void ScenarioSelfVote(const common::TrajectoryPoint& ego_point,
  //                       const Frame& frame);
  // bool ReuseCurrentScenario(const common::TrajectoryPoint& ego_point,
  //                           const Frame& frame);
  // bool SelectScenario(const ScenarioConfig::ScenarioType type,
  //                     const common::TrajectoryPoint& ego_point,
  //                     const Frame& frame);

  void ScenarioDispatch(const common::TrajectoryPoint& ego_point,
                        const Frame& frame);

  bool IsBareIntersectionScenario(
      const ScenarioConfig::ScenarioType& scenario_type);
  bool IsStopSignScenario(const ScenarioConfig::ScenarioType& scenario_type);
  bool IsTrafficLightScenario(
      const ScenarioConfig::ScenarioType& scenario_type);

  void UpdatePlanningContext(const Frame& frame,
                             const ScenarioConfig::ScenarioType& scenario_type);

  void UpdatePlanningContextBareIntersectionScenario(
      const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type);

  void UpdatePlanningContextStopSignScenario(
      const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type);

  void UpdatePlanningContextTrafficLightScenario(
      const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type);

  void UpdatePlanningContextPullOverScenario(
      const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type);

 private:
  std::unordered_map<ScenarioConfig::ScenarioType, ScenarioConfig,
                     std::hash<int>>
      config_map_;
  std::unique_ptr<Scenario> current_scenario_;
  ScenarioConfig::ScenarioType default_scenario_type_;
  std::set<ScenarioConfig::ScenarioType> supported_scenarios_;
  ScenarioContext scenario_context_;
  std::unordered_map<ReferenceLineInfo::OverlapType, hdmap::PathOverlap,
                     std::hash<int>>
      first_encountered_overlap_map_;
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
