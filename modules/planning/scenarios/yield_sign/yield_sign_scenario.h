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
#include <unordered_map>
#include <vector>

#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/planning/scenarios/yield_sign/proto/yield_sign.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/util/factory.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

namespace apollo {
namespace planning {

// stage context
struct YieldSignContext : public ScenarioContext {
  ScenarioYieldSignConfig scenario_config;
  std::vector<std::string> current_yield_sign_overlap_ids;
  double creep_start_time = 0.0;
};

class YieldSignScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector,
            const std::string& name) override;

  /**
   * @brief Get the scenario context.
   */
  YieldSignContext* GetContext() override { return &context_; }

  bool IsTransferable(const Scenario* const other_scenario,
                      const Frame& frame) override;

  bool Exit(Frame* frame) override;

  bool Enter(Frame* frame) override;

 private:
  bool init_ = false;
  YieldSignContext context_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::YieldSignScenario,
                                     Scenario)

}  // namespace planning
}  // namespace apollo
