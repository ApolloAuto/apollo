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
#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"
#include "modules/planning/scenarios/large_curvature/proto/large_curvature_scenario.pb.h"

namespace apollo {
namespace planning {

struct LargeCurvatureContext : public ScenarioContext {
    ScenarioLargeCurvatureConfig scenario_config;
};

class LargeCurvatureScenario : public Scenario {
public:
    bool Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) override;

    /**
     * @brief Get the scenario context.
     */
    LargeCurvatureContext* GetContext() override {
        return &context_;
    }

    bool IsTransferable(const Scenario* const other_scenario, const Frame& frame) override;

private:
    bool init_ = false;
    LargeCurvatureContext context_;
    const hdmap::HDMap* hdmap_ = nullptr;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LargeCurvatureScenario, Scenario)

}  // namespace planning
}  // namespace apollo
