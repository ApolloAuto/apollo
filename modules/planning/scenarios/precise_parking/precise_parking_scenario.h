/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common_msgs/external_command_msgs/precise_parking_command.pb.h"
#include "modules/planning/scenarios/precise_parking/proto/precise_parking_scenario.pb.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

namespace apollo {
namespace planning {

struct PreciseParkingContext : public ScenarioContext {
    ScenarioPreciseParkingConfig scenario_config;
    apollo::external_command::PreciseParkingCommand precise_parking_command;
};

class PreciseParkingScenario : public Scenario {
public:
    bool Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) override;

    /**
     * @brief Get the scenario context.
     */
    PreciseParkingContext* GetContext() override {
        return &context_;
    }

    bool IsTransferable(const Scenario* const other_scenario, const Frame& frame) override;

private:
    void Rerouting(const localization::Pose& start_pose,
                   const localization::Pose& target_pose);
    PreciseParkingContext context_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::PreciseParkingScenario, Scenario)

}  // namespace planning
}  // namespace apollo
