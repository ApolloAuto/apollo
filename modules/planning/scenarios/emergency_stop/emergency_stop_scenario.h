/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/emergency_stop/proto/emergency_stop.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/util/factory.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

namespace apollo {
namespace planning {

// stage context
struct EmergencyStopContext : public ScenarioContext {
  ScenarioEmergencyStopConfig scenario_config;
};

class EmergencyStopScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector,
            const std::string& name) override;

  /**
   * @brief Get the scenario context.
   */
  EmergencyStopContext* GetContext() override { return &context_; }

  bool IsTransferable(const Scenario* const other_scenario,
                      const Frame& frame) override;

  ScenarioResult Process(const common::TrajectoryPoint& planning_init_point,
                         Frame* frame) override;

  bool Exit(Frame* frame);

 private:
  bool init_ = false;
  EmergencyStopContext context_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::EmergencyStopScenario,
                                     Scenario)

}  // namespace planning
}  // namespace apollo
