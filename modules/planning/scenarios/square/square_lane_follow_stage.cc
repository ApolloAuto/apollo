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

#include "modules/planning/scenarios/square/square_lane_follow_stage.h"

#include <algorithm>
#include <utility>
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/scenarios/square/context.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

bool SquareLaneFollowStage::Init(
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
    return ret;
}

StageResult SquareLaneFollowStage::Process(const common::TrajectoryPoint& planning_start_point, Frame* frame) {
    AINFO << "In SquareLaneFollowStage";
    if (frame->reference_line_info().empty()) {
        return StageResult(StageStatusType::FINISHED);
    }
    apollo::common::VehicleState vehicle_state = injector_->vehicle_state()->vehicle_state();
    if (vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE) {
        frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
        auto* traj_pair = frame->mutable_open_space_info()->mutable_publishable_trajectory_data();
        static constexpr double gear_shift_max_t = 5.0;
        static constexpr double gear_shift_unit_t = 0.1;
        for (double t = 0.0; t < gear_shift_max_t; t += gear_shift_unit_t) {
            common::TrajectoryPoint point;
            point.mutable_path_point()->set_x(frame->vehicle_state().x());
            point.mutable_path_point()->set_y(frame->vehicle_state().y());
            point.mutable_path_point()->set_theta(frame->vehicle_state().heading());
            point.mutable_path_point()->set_s(0.0);
            point.mutable_path_point()->set_kappa(0);
            point.set_relative_time(t);
            point.set_v(0.0);
            point.set_a(0.0);
            traj_pair->first.emplace_back(point);
        }
        traj_pair->second = apollo::canbus::Chassis::GEAR_DRIVE;
        AINFO << "GEAR SHIFT TO DRIVE";
        return StageResult(StageStatusType::RUNNING);
    }
    std::string junction_id = GetContextAs<SquareContext>()->junction_id;
    StageResult result = ExecuteTaskOnReferenceLine(planning_start_point, frame);
    if (result.HasError()) {
        AERROR << "SquareLaneFollowStage planning error";
    }
    const auto& reference_line_info = frame->reference_line_info().back();
    hdmap::PathOverlap* current_pnc_junction
            = reference_line_info.GetOverlapOnReferenceLine(junction_id, ReferenceLineInfo::JUNCTION);
    double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
    if (!current_pnc_junction || current_pnc_junction->end_s < adc_end_s) {
        return FinishScenario();
    }
    const auto& path_data = reference_line_info.path_data();
    bool is_path_blocked = false;
    if (path_data.path_label().find("regular") == std::string::npos) {
        is_path_blocked = true;
    } else if (!path_data.blocking_obstacle_id().empty()) {
        is_path_blocked = true;
    } else {
        is_path_blocked = false;
    }
    bool is_square_path_block = is_path_blocked && vehicle_state.linear_velocity() < 0.1;

    double trajectory_length = 0;
    if (reference_line_info.trajectory().size() > 0) {
        trajectory_length = reference_line_info.trajectory().back().path_point().s()
                - reference_line_info.trajectory().front().path_point().s();
    }
    is_square_path_block = is_square_path_block
            || (path_data.path_label().find("square") != std::string::npos && vehicle_state.linear_velocity() < 0.1
                && trajectory_length < 0.1);
    if (is_square_path_block) {
        blocking_times_ = std::min<int>(blocking_times_ + 1, 50);
    } else {
        blocking_times_ = 0;
    }
    AINFO << "label: " << path_data.path_label() << ", blocking_obstacle_id: " << path_data.blocking_obstacle_id()
          << ", is_path_blocked: " << is_path_blocked
          << ", vehicle_state.linear_velocity: " << vehicle_state.linear_velocity()
          << ", blocking_times_: " << blocking_times_;
    if (blocking_times_ >= 50) {
        auto* context = GetContextAs<SquareContext>();
        context->blocking_obstacle_id = path_data.blocking_obstacle_id();
        return FinishStage();
    }
    return StageResult(StageStatusType::RUNNING);
}
StageResult SquareLaneFollowStage::FinishStage() {
    next_stage_ = "EXTRICATE_STAGE";
    return StageResult(StageStatusType::FINISHED);
}
}  // namespace planning
}  // namespace apollo
