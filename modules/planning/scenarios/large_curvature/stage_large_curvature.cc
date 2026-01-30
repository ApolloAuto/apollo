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

#include "modules/planning/scenarios/large_curvature/stage_large_curvature.h"

#include <string>
#include <vector>

#include "modules/planning/planning_open_space/utils/open_space_roi_util.h"

namespace apollo {
namespace planning {

bool StageLargeCurvature::Init(
        const StagePipeline& config,
        const std::shared_ptr<DependencyInjector>& injector,
        const std::string& config_dir,
        void* context) {
    CHECK_NOTNULL(context);
    bool ret = Stage::Init(config, injector, config_dir, context);
    if (!ret) {
        AERROR << Name() << "init failed!";
        return false;
    }
    scenario_config_.CopyFrom(GetContextAs<LargeCurvatureContext>()->scenario_config);
    return ret;
}

StageResult StageLargeCurvature::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
    frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
    StageResult result = ExecuteTaskOnOpenSpace(frame);
    if (result.HasError()) {
        AERROR << "StageLargeCurvature planning error";
        return result.SetStageStatus(StageStatusType::ERROR);
    }

    if (frame->open_space_info().destination_reached() || IsCurvatureSmall(frame)) {
        AINFO << "StageLargeCurvature destination reached";
        next_stage_.clear();
        return result.SetStageStatus(StageStatusType::FINISHED);
    }
    return result.SetStageStatus(StageStatusType::RUNNING);
}

bool StageLargeCurvature::IsCurvatureSmall(Frame* frame) {
    if (frame->reference_line_info().empty()) {
        AINFO << "StageLargeCurvature: reference line info is empty";
        return false;
    }

    common::VehicleState vehicle_state = injector_->vehicle_state()->vehicle_state();
    common::SLPoint adc_sl;
    if (!frame->reference_line_info().front().reference_line().XYToSL(
                {vehicle_state.x(), vehicle_state.y()}, &adc_sl)) {
        AINFO << "StageLargeCurvature: failed to get nearest lane";
        return false;
    }

    double check_s = 0;
    double kappa = 0;
    for (double delta_s = 0; delta_s < scenario_config_.curvature_check_distance();
         delta_s += scenario_config_.curvature_check_delta_s()) {
        check_s = adc_sl.s() + delta_s;
        if (frame->reference_line_info().front().reference_line().GetMapPath().accumulated_s().back() + 1e-2
            < check_s) {
            AINFO << "StageLargeCurvature: s is too large";
            return false;
        }

        kappa = frame->reference_line_info().front().reference_line().GetNearestReferencePoint(check_s).kappa();
        if (fabs(kappa) > scenario_config_.curvature_check_max()) {
            AINFO << "StageLargeCurvature: curvature is too large";
            return false;
        }
        AINFO << "delta_s: " << delta_s << " check_s: " << check_s << " kappa: " << kappa << " point: "
              << frame->reference_line_info().front().reference_line().GetNearestReferencePoint(check_s).DebugString();
    }
    return true;
}

}  // namespace planning
}  // namespace apollo
