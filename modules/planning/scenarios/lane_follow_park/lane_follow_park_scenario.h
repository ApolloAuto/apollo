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

/**
 * @file
 **/

#pragma once

#include <memory>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/planning/scenarios/lane_follow_park/proto/lane_follow_park_scenario.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"
#include "modules/planning/planning_base/reference_line/reference_point.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

namespace apollo {
namespace planning {

struct LaneFollowParkContext : public ScenarioContext {
  ScenarioLaneFollowParkConfig scenario_config;
};

class LaneFollowParkScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) override;
  /**
   * @brief Get the scenario context.
   */
  ScenarioContext* GetContext() override { return &context_; }

  bool IsTransferable(const Scenario* other_scenario,
                      const Frame& frame) override;
  
 private:
  LaneFollowParkContext context_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LaneFollowParkScenario,
                                     Scenario)

}  // namespace planning
}  // namespace apollo
