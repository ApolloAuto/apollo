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

#include "modules/planning/scenarios/precise_parking/stage_approaching_precise_parking.h"

namespace apollo {
namespace planning {

using apollo::hdmap::ParkingSpaceInfoConstPtr;
using common::math::Polygon2d;
using common::math::Vec2d;

bool StageApproachingPreciseParking::Init(
        const StagePipeline& config,
        const std::shared_ptr<DependencyInjector>& injector,
        const std::string& config_dir,
        void* context) {
    if (!Stage::Init(config, injector, config_dir, context)) {
        return false;
    }
    scenario_context_ = *(GetContextAs<PreciseParkingContext>());
    // hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
    // CHECK_NOTNULL(hdmap_);
    return true;
}
StageResult StageApproachingPreciseParking::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
    ADEBUG << "stage: StageApproachingPreciseParking";
    CHECK_NOTNULL(frame);
    StageResult result;

    if (frame->reference_line_info().empty()) {
        AERROR << "frame has no reference line";
        return StageResult(StageStatusType::ERROR);
    }

    BuildPreStopObs(frame);
    result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

    if (CheckADCInPreciseParkingRange(*frame) && CheckADCStop(*frame)) {
        next_stage_ = "PRECISE_PARKING";
        return StageResult(StageStatusType::FINISHED);
    }
    if (result.HasError()) {
        AERROR << "StageApproachingPreciseParking planning error";
        return result.SetStageStatus(StageStatusType::ERROR);
    }

    return result.SetStageStatus(StageStatusType::RUNNING);
}

void StageApproachingPreciseParking::BuildPreStopObs(Frame* frame) {
  common::SLPoint pre_stop_point;
  Vec2d target_pos(
    scenario_context_.precise_parking_command.parking_spot_pose().x(),
    scenario_context_.precise_parking_command.parking_spot_pose().y());
    frame->reference_line_info().front().reference_line().XYToSL(target_pos, &pre_stop_point);

    std::string stop_wall_id = "precise_parking_pre_stop";
    std::vector<std::string> wait_for_obstacles;
    util::BuildStopDecision(stop_wall_id, pre_stop_point.s(), 0.0,
                          StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                          wait_for_obstacles, "OpenSpacePreStopDecider", frame,
                          &(frame->mutable_reference_line_info()->front()));
}

bool StageApproachingPreciseParking::CheckADCStop(const Frame& frame) {
    const double adc_speed = injector_->vehicle_state()->linear_velocity();
    const double max_adc_stop_speed
            = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().max_abs_speed_when_stopped();
    if (adc_speed > max_adc_stop_speed) {
        ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
        return false;
    }
    return true;
}

bool StageApproachingPreciseParking::CheckADCInPreciseParkingRange(const Frame& frame) {
    // check stop close enough to stop line of the stop_sign
    const double adc_front_edge_s = frame.reference_line_info().front().AdcSlBoundary().end_s();
    const double adc_end_edge_s = frame.reference_line_info().front().AdcSlBoundary().start_s();

    // std::string junction_id = scenario_context->precise_parking_command.junction_id();
    // hdmap::JunctionInfoConstPtr junction_ptr = hdmap_->GetJunctionById(hdmap::MakeMapId(junction_id));

    common::PointENU point;
    std::vector<hdmap::AreaInfoConstPtr> areas;
    double target_heading = scenario_context_.precise_parking_command.parking_spot_pose().heading();
    point.set_x(scenario_context_.precise_parking_command.parking_spot_pose().x());
    point.set_y(scenario_context_.precise_parking_command.parking_spot_pose().y());
    if (hdmap::HDMapUtil::BaseMapPtr()->GetAreas(point, 1e-1, &areas) < 0 || areas.empty()) {
        AERROR << "can not find any junction near parking spot";
        return false;
    };

    double ego_width = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().width();
    double ego_length = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().length();
    Vec2d shift_vec = (ego_length / 2 - common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().back_edge_to_center()) * Vec2d::CreateUnitVec2d(target_heading);
    Vec2d center(point.x(), point.y());
    center = center + shift_vec;
    common::math::Box2d target_box(center, target_heading, ego_length, ego_width);
    if (areas.front()->polygon().Contains(Polygon2d(injector_->ego_info()->ego_box())) &&
        areas.front()->polygon().Contains(Polygon2d(target_box))) {
        AINFO << "ADC and target in precise parking range";
        return true;
    }
    return false;
}

}  // namespace planning
}  // namespace apollo
