/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PLANNING_LATTICE_BEHAVIOR_SCENARIO_MANAGER_H_
#define MODULES_PLANNING_LATTICE_BEHAVIOR_SCENARIO_MANAGER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/planning/lattice/behavior/scenario.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {

class ScenarioManager {
 private:
  enum FeatureLevel {
    LEVEL0 = 0,
    LEVEL1,
    LEVEL2,
    LEVEL3,
    NUM_LEVELS,
  };

 public:
  void Reset();
  int ComputeWorldDecision(Frame* frame,
                           ReferenceLineInfo* const reference_line_info,
                           PlanningTarget* planning_target);

  template <class T>
  void RegisterScenario(FeatureLevel level) {
    auto scenario = std::unique_ptr<T>(new T());
    scenarios_[static_cast<int>(level)].push_back(scenario.get());
    indexed_scenarios_[scenario->Name()] = std::move(scenario);
  }

  template <class T>
  const T* FindScenario() const {
    auto scenario_iter = indexed_scenarios_.find(T::scenario_name());
    if (scenario_iter == indexed_scenarios_.end()) {
      return nullptr;
    } else {
      return dynamic_cast<const T*>(scenario_iter->second.get());
    }
  }

 private:
  void RegisterScenarios();
  std::vector<std::vector<Scenario*>> scenarios_;
  std::unordered_map<std::string, std::unique_ptr<Scenario>> indexed_scenarios_;
  bool initialized_ = false;
  DECLARE_SINGLETON(ScenarioManager);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_BEHAVIOR_SCENARIO_MANAGER_H_
