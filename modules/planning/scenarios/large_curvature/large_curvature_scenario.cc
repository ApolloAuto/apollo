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

#include "modules/planning/scenarios/large_curvature/large_curvature_scenario.h"

#include <vector>

#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_open_space/utils/open_space_roi_util.h"
#include "modules/planning/scenarios/large_curvature/stage_large_curvature.h"

namespace apollo {
namespace planning {
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

bool LargeCurvatureScenario::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) {
    // hook: Apollo License Verification: v_apollo_park
    if (init_) {
        return true;
    }

    if (!Scenario::Init(injector, name)) {
        AERROR << "failed to init scenario" << Name();
        return false;
    }

    if (!Scenario::LoadConfig<ScenarioLargeCurvatureConfig>(&context_.scenario_config)) {
        AERROR << "fail to get config of scenario" << Name();
        return false;
    }
    AINFO << context_.scenario_config.DebugString();
    hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
    CHECK_NOTNULL(hdmap_);
    init_ = true;
    return true;
}

bool LargeCurvatureScenario::IsTransferable(const Scenario* const other_scenario, const Frame& frame) {
    if (!frame.local_view().planning_command->has_lane_follow_command()) {
        AINFO << "No lane follow command from routing";
        return false;
    }
    if (other_scenario == nullptr || frame.reference_line_info().empty()) {
        AINFO << "Other scenario is null or no reference line";
        return false;
    }

    // check if ego in big curvature
    common::VehicleState vehicle_state = injector_->vehicle_state()->vehicle_state();
    common::SLPoint adc_sl;
    if (!frame.reference_line_info().front().reference_line().XYToSL({vehicle_state.x(), vehicle_state.y()}, &adc_sl)) {
        AINFO << "scenario LargeCurvature: failed to get nearest lane";
        return false;
    }

    double kappa
            = frame.reference_line_info().front().reference_line().GetNearestReferencePoint(adc_sl.s() + 3.0).kappa();
    AINFO << "kappa: " << kappa << " ego_x : " << vehicle_state.x() << " ego_y: " << vehicle_state.y()
          << " ref length: " << frame.reference_line_info().front().reference_line().Length() << " s: " << adc_sl.s();
    double vehicle_max_curvature = 1 / common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().min_turn_radius();
    AINFO << vehicle_max_curvature * context_.scenario_config.curvature_ratio();
    if (fabs(kappa) > vehicle_max_curvature * context_.scenario_config.curvature_ratio()
        && frame.reference_line_info().front().SDistanceToDestination() > 10.0) {
        AINFO << "ego in large curvature";
        return true;
    }
    return false;
}  // namespace planning

}  // namespace planning
}  // namespace apollo
