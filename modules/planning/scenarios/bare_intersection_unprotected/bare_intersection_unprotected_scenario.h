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

#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/planning/scenarios/bare_intersection_unprotected/proto/bare_intersection_unprotected_scenario.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/util/factory.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

namespace apollo {
namespace planning {

// stage context
struct BareIntersectionUnprotectedContext : public ScenarioContext {
  ScenarioBareIntersectionUnprotectedConfig scenario_config;
  std::string current_pnc_junction_overlap_id;
};

class BareIntersectionUnprotectedScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector,
            const std::string& name) override;
  /**
   * @brief Get the scenario context.
   */
  BareIntersectionUnprotectedContext* GetContext() override {
    return &context_;
  }

  bool IsTransferable(const Scenario* other_scenario,
                      const Frame& frame) override;

  bool Enter(Frame* frame) override;

 private:
  bool init_ = false;
  BareIntersectionUnprotectedContext context_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::BareIntersectionUnprotectedScenario, Scenario)

}  // namespace planning
}  // namespace apollo
