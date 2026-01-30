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

#include "modules/planning/scenarios/square/extricate_stage.h"

#include <string>
#include <vector>
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/planning_open_space/utils/open_space_roi_util.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
#define AINFO AERROR
bool ExtricateStage::Init(
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
    scenario_config_.CopyFrom(GetContextAs<SquareContext>()->scenario_config);
    return ret;
}

StageResult ExtricateStage::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
    AINFO << "In ExtricateStage";
    apollo::common::VehicleState vehicle_state = injector_->vehicle_state()->vehicle_state();
    if (vehicle_state.gear() != canbus::Chassis::GEAR_REVERSE) {
        frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
        auto* traj_pair = frame->mutable_open_space_info()->mutable_publishable_trajectory_data();
        static constexpr double gear_shift_max_t = 5.0;
        static constexpr double gear_shift_unit_t = 0.1;
        for (double t = 0.0; t < gear_shift_max_t; t += gear_shift_unit_t) {
            TrajectoryPoint point;
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
        traj_pair->second = apollo::canbus::Chassis::GEAR_REVERSE;
        AINFO << "GENERATE REVERSE TRAJECTORY";
        need_to_stop_ = false;
        return StageResult(StageStatusType::RUNNING);
    }
    AINFO << "gear is reverse";
    auto& reference_line_info = frame->mutable_reference_line_info()->back();
    const auto& reference_line = reference_line_info.reference_line();
    double adc_start_s = reference_line_info.AdcSlBoundary().start_s();
    // GenerateStraightReversePath(reference_line_info.mutable_path_data(),
    //                             planning_init_point);
    if (stop_point_ == nullptr) {
        apollo::common::math::Vec2d stop_point;
        auto ref_point = reference_line.GetReferencePoint(adc_start_s - 10.0);
        stop_point_ = std::make_shared<common::PathPoint>(ref_point.ToPathPoint(0.0));
    }
    AINFO << "need to stop";
    common::SLPoint sl_point;
    reference_line.XYToSL(*stop_point_, &sl_point);
    std::vector<std::string> wait_for_obstacles;
    planning::util::BuildStopDecision(
            "reverse_stop",
            sl_point.s(),
            0.1,
            StopReasonCode::STOP_REASON_STOP_SIGN,
            wait_for_obstacles,
            "reverse stop",
            frame,
            &(frame->mutable_reference_line_info()->front()));
    StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
    const auto& candidate_paths = reference_line_info.GetCandidatePathData();
    bool has_regular_path = false;
    for (const auto& path_data : candidate_paths) {
        if (path_data.path_label().find("regular") != std::string::npos && path_data.blocking_obstacle_id().empty()) {
            AINFO << "find regular path: " << path_data.path_label();
            has_regular_path = true;
            break;
        }
    }
    if (has_regular_path) {
        success_path_times_++;
    } else {
        success_path_times_ = 0;
        need_to_stop_ = false;
    }
    if (success_path_times_ > 10 && !need_to_stop_) {
        AINFO << "find square path change stage";
        need_to_stop_ = true;
        *stop_point_ = reference_line_info.mutable_path_data()->discretized_path().Evaluate(5.0);
        AINFO << "stop_point_:" << stop_point_->x() << "," << stop_point_->y();
    }
    if (success_path_times_ > 10 && std::fabs(vehicle_state.linear_velocity()) < 0.1) {
        AINFO << "already stopped change stage" << vehicle_state.linear_velocity() << " " << vehicle_state.x() << ", "
              << vehicle_state.y();
        return FinishStage();
    }
    return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult ExtricateStage::FinishStage() {
    next_stage_ = "SQUARE_LANE_FOLLOW_STAGE";
    success_path_times_ = 0;
    need_to_stop_ = false;
    return StageResult(StageStatusType::FINISHED);
}

bool ExtricateStage::HasCollision(Frame* frame, const common::math::Box2d& adc_box) {
    auto obstacles_by_frame = frame->GetObstacleList();
    common::math::Polygon2d adc_polygon(adc_box);
    for (const auto& obstacle : obstacles_by_frame->Items()) {
        if (adc_polygon.HasOverlap(obstacle->PerceptionPolygon())) {
            return true;
        }
    }
    return false;
}

bool ExtricateStage::GenerateStraightReversePath(PathData* path_data, const common::TrajectoryPoint& init_point) {
    DiscretizedPath path;
    std::vector<common::PathPoint> path_points;
    static constexpr double kReversePathLength = 10.0;
    static constexpr double kReversePathUnit = 0.5;
    for (double s = 0; s <= kReversePathLength; s += kReversePathUnit) {
        common::PathPoint point;
        point.set_x(init_point.path_point().x() - s * std::cos(init_point.path_point().theta()));
        point.set_y(init_point.path_point().y() - s * std::sin(init_point.path_point().theta()));
        point.set_theta(init_point.path_point().theta());
        point.set_s(s);
        point.set_kappa(0.0);
        point.set_dkappa(0.0);
        point.set_ddkappa(0.0);
        path.emplace_back(point);
    }
    path_data->SetDiscretizedPath(path);
    path_data->set_path_label("reverse_straight_path");
    path_data->set_is_reverse_path(true);
    return true;
}

}  // namespace planning
}  // namespace apollo
