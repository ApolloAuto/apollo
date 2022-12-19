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
#include <unordered_map>

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {
namespace scenario {

class ScenarioManager final {
 public:
  ScenarioManager() = delete;

  explicit ScenarioManager(const std::shared_ptr<DependencyInjector>& injector);

  bool Init(const PlanningConfig& planning_config);

  Scenario* mutable_scenario() { return current_scenario_.get(); }

  DependencyInjector* injector() { return injector_.get(); }

  void Update(const common::TrajectoryPoint& ego_point, const Frame& frame);

 private:
  void Observe(const Frame& frame);

  std::unique_ptr<Scenario> CreateScenario(ScenarioType scenario_type);

  void RegisterScenarios();

  ScenarioType SelectBareIntersectionScenario(
      const Frame& frame, const hdmap::PathOverlap& pnc_junction_overlap);

  ScenarioType SelectPullOverScenario(const Frame& frame);

  ScenarioType SelectPadMsgScenario(const Frame& frame);

  ScenarioType SelectInterceptionScenario(const Frame& frame);

  ScenarioType SelectStopSignScenario(
      const Frame& frame, const hdmap::PathOverlap& stop_sign_overlap);

  ScenarioType SelectTrafficLightScenario(
      const Frame& frame, const hdmap::PathOverlap& traffic_light_overlap);

  ScenarioType SelectValetParkingScenario(const Frame& frame);

  ScenarioType SelectYieldSignScenario(
      const Frame& frame, const hdmap::PathOverlap& yield_sign_overlap);

  ScenarioType SelectParkAndGoScenario(const Frame& frame);

  void ScenarioDispatch(const Frame& frame);
  ScenarioType ScenarioDispatchLearning();
  ScenarioType ScenarioDispatchNonLearning(const Frame& frame);

  bool IsBareIntersectionScenario(const ScenarioType& scenario_type);
  bool IsStopSignScenario(const ScenarioType& scenario_type);
  bool IsTrafficLightScenario(const ScenarioType& scenario_type);
  bool IsYieldSignScenario(const ScenarioType& scenario_type);

  void UpdatePlanningContext(const Frame& frame,
                             const ScenarioType& scenario_type);

  void UpdatePlanningContextBareIntersectionScenario(
      const Frame& frame, const ScenarioType& scenario_type);

  void UpdatePlanningContextEmergencyStopcenario(
      const Frame& frame, const ScenarioType& scenario_type);

  void UpdatePlanningContextPullOverScenario(const Frame& frame,
                                             const ScenarioType& scenario_type);

  void UpdatePlanningContextStopSignScenario(const Frame& frame,
                                             const ScenarioType& scenario_type);

  void UpdatePlanningContextTrafficLightScenario(
      const Frame& frame, const ScenarioType& scenario_type);

  void UpdatePlanningContextYieldSignScenario(
      const Frame& frame, const ScenarioType& scenario_type);

 private:
  std::shared_ptr<DependencyInjector> injector_;
  PlanningConfig planning_config_;
  std::unordered_map<ScenarioType, ScenarioConfig, std::hash<int>> config_map_;
  std::unique_ptr<Scenario> current_scenario_;
  ScenarioType default_scenario_type_;
  ScenarioContext scenario_context_;
  std::unordered_map<ReferenceLineInfo::OverlapType, hdmap::PathOverlap,
                     std::hash<int>>
      first_encountered_overlap_map_;
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
