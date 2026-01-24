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

#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_open_space/utils/open_space_roi_util.h"
#include "modules/planning/scenarios/precise_parking/precise_parking_scenario.h"

namespace apollo {
namespace planning {
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::external_command::PreciseParkingCommand;

bool PreciseParkingScenario::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) {
    if (!Scenario::Init(injector, name)) {
        AERROR << "failed to init scenario" << Name();
        return false;
    }

    if (!Scenario::LoadConfig<ScenarioPreciseParkingConfig>(&context_.scenario_config)) {
        AERROR << "fail to get config of scenario" << Name();
        return false;
    }
    return true;
}

bool PreciseParkingScenario::IsTransferable(const Scenario* const other_scenario, const Frame& frame) {
    const auto& planning_command = frame.local_view().planning_command;
    if (!planning_command->has_custom_command()) {
        return false;
    }
    if (!planning_command->custom_command().Is<PreciseParkingCommand>()) {
        return false;
    }
    context_.precise_parking_command.Clear();
    if (!planning_command->custom_command().UnpackTo(&context_.precise_parking_command)) {
        AERROR << "Precise Parking Command unpack error" << planning_command->DebugString();
        return false;
    }
    return true;
}  // namespace planning

}  // namespace planning
}  // namespace apollo
