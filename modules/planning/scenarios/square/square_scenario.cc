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

#include <string>
#include "cyber/common/log.h"
#include "modules/planning/scenarios/square/square_scenario.h"

namespace apollo {
namespace planning {

bool SquareScenario::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) {
    if (!Scenario::Init(injector, name)) {
        AERROR << "failed to init scenario" << Name();
        return false;
    }

    if (!Scenario::LoadConfig<apollo::planning::ScenarioSquareConfig>(&context_.scenario_config)) {
        AERROR << "fail to get config of scenario" << Name();
        return false;
    }

    return true;
}

bool SquareScenario::IsTransferable(const Scenario* other_scenario, const Frame& frame) {
    if (!frame.local_view().planning_command->has_lane_follow_command()) {
        return false;
    }
    if (frame.reference_line_info().empty() || frame.reference_line_info().size() > 1) {
        return false;
    }
    const auto& reference_line_info = frame.reference_line_info().front();
    hdmap::PathOverlap junction_overlap;
    if (reference_line_info.GetJunction(reference_line_info.AdcSlBoundary().end_s(), &junction_overlap) == 0) {
        return false;
    }
    if (reference_line_info.AdcSlBoundary().start_s() < junction_overlap.start_s
        || reference_line_info.AdcSlBoundary().end_s() > junction_overlap.end_s) {
        return false;
    }
    context_.junction_id = junction_overlap.object_id;
    AERROR << "JUNCTION ID" << context_.junction_id;
    return true;
}

}  // namespace planning
}  // namespace apollo
