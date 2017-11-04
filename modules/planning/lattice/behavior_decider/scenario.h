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
 * @file scenario.h
 **/



#ifndef MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_SCENARIO_H_
#define MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_SCENARIO_H_

#include <string>
#include <memory>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/lattice_sampling_config.pb.h"

#define DECLARE_SCENARIO(WORLD) \
 public: \
  WORLD() : SCENARIO(#WORLD) {}; \
  static std::string FeatureName() { return #WORLD; } \
 private: \
  WORLD(const WORLD&) = delete; \
  WORLD& operator = (const WORLD&) = delete

namespace apollo {
namespace planning {

//using ObjectId = std::string;
//using ObjectIds = std::vector<ObjectId>;

class Scenario {
 public:
  explicit Scenario(std::string name) : _name(std::move(name)) {}
  virtual ~Scenario() = default;
  /**
   * Individual Decision/SamplingCondition for specific scenario
   * @return 0 if success
   */
  virtual int ComputeScenarioDecision(
      const std::vector<common::PathPoint>& discretized_reference_line,
      std::vector<PlanningTarget*>* const decisions) = 0;

  virtual const std::string& Name() const { return name_; }
  /**
   * whether this scenario exists in current condition.
   */
  virtual bool ScenarioExist() const = 0;
  /**
   * reset is called before construction
   */
  virtual void Reset() = 0;

 private:
    std::string name_;
};

}  // namespace apollo
}  // namespace planning

#endif  // MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_SCENARIO_H_
