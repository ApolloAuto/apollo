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
#include <string>
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"
#include "modules/planning/scenarios/square/context.h"

namespace apollo {
namespace planning {

class SquareScenario : public Scenario {
public:
    bool Init(std::shared_ptr<DependencyInjector> injector, const std::string& name);
    /**
     * @brief Get the scenario context.
     */
    SquareContext* GetContext() override {
        return &context_;
    }

    bool IsTransferable(const Scenario* other_scenario, const Frame& frame) override;

private:
    SquareContext context_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::SquareScenario, Scenario)

}  // namespace planning
}  // namespace apollo
