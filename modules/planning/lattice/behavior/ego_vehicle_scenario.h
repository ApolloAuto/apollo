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

#ifndef MODULES_PLANNING_LATTICE_BEHAVIOR_EGO_VEHICLE_SCENARIO_H_
#define MODULES_PLANNING_LATTICE_BEHAVIOR_EGO_VEHICLE_SCENARIO_H_

#include "modules/planning/lattice/behavior/scenario.h"

namespace apollo {
namespace planning {

class EgoVehicleScenario : public Scenario {
 public:
  void Reset() override;

  bool Init() override;

  bool ScenarioExist() const override { return exist_; }

  virtual int ComputeScenarioDecision(
      Frame* frame, ReferenceLineInfo* const reference_line_info,
      PlanningTarget* const decision);

 private:
  bool exist_ = false;

  DECLARE_SCENARIO(EgoVehicleScenario);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_BEHAVIOR_EGO_VEHICLE_SCENARIO_H_
