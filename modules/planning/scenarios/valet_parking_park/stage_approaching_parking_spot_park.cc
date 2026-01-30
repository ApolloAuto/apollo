/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/scenarios/valet_parking_park/stage_approaching_parking_spot_park.h"

namespace apollo {
namespace planning {

using apollo::hdmap::ParkingSpaceInfoConstPtr;
using common::math::Vec2d;

bool StageApproachingParkingSpotPark::Init(
        const StagePipeline& config,
        const std::shared_ptr<DependencyInjector>& injector,
        const std::string& config_dir,
        void* context) {
    if (!Stage::Init(config, injector, config_dir, context)) {
        return false;
    }
    scenario_config_.CopyFrom(GetContextAs<ValetParkingContext>()->scenario_config);
    return true;
}
StageResult StageApproachingParkingSpotPark::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
    ADEBUG << "stage: StageApproachingParkingSpot";
    CHECK_NOTNULL(frame);
    StageResult result;
    auto scenario_context = GetContextAs<ValetParkingContext>();

    if (scenario_context->target_parking_spot_id.empty()) {
        return result.SetStageStatus(StageStatusType::ERROR);
    }

    *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) = scenario_context->target_parking_spot_id;
    frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(scenario_context->pre_stop_rightaway_flag);
    *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point())
            = scenario_context->pre_stop_rightaway_point;
    auto* reference_lines = frame->mutable_reference_line_info();
    for (auto& reference_line : *reference_lines) {
        auto* path_decision = reference_line.path_decision();
        if (nullptr == path_decision) {
            continue;
        }
        auto* dest_obstacle = path_decision->Find(FLAGS_destination_obstacle_id);
        if (nullptr == dest_obstacle) {
            continue;
        }
        ObjectDecisionType decision;
        decision.mutable_ignore();
        dest_obstacle->EraseDecision();
        dest_obstacle->AddLongitudinalDecision("ignore-dest-in-valet-parking", decision);
    }
    result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

    scenario_context->pre_stop_rightaway_flag = frame->open_space_info().pre_stop_rightaway_flag();
    scenario_context->pre_stop_rightaway_point = frame->open_space_info().pre_stop_rightaway_point();

    if (CheckADCStop(*frame) && CheckADCInParkingRange(*frame)) {
        next_stage_ = "VALET_PARKING_PARKING_PARK";
        return StageResult(StageStatusType::FINISHED);
    }
    if (result.HasError()) {
        AERROR << "StopSignUnprotectedStagePreStop planning error";
        return result.SetStageStatus(StageStatusType::ERROR);
    }

    return result.SetStageStatus(StageStatusType::RUNNING);
}

bool StageApproachingParkingSpotPark::CheckADCStop(const Frame& frame) {
    const double adc_speed = injector_->vehicle_state()->linear_velocity();
    const double max_adc_stop_speed
            = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().max_abs_speed_when_stopped();
    if (adc_speed > max_adc_stop_speed) {
        ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
        return false;
    }
    return true;
}

double StageApproachingParkingSpotPark::GetTargetS(const Frame& frame) {
    const auto& target_parking_spot_id = frame.open_space_info().target_parking_spot_id();
    const auto& nearby_path = frame.reference_line_info().front().reference_line().map_path();
    if (target_parking_spot_id.empty()) {
        AERROR << "no target parking spot id found when setting pre stop fence";
        return false;
    }

    double target_area_center_s = 0.0;
    const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
    ParkingSpaceInfoConstPtr target_parking_spot_ptr;
    const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
    for (const auto& parking_overlap : parking_space_overlaps) {
        if (parking_overlap.object_id == target_parking_spot_id) {
            hdmap::Id id;
            id.set_id(parking_overlap.object_id);
            target_parking_spot_ptr = hdmap->GetParkingSpaceById(id);
            Vec2d left_bottom_point = target_parking_spot_ptr->polygon().points().at(0);
            Vec2d right_bottom_point = target_parking_spot_ptr->polygon().points().at(1);
            Vec2d right_up_point = target_parking_spot_ptr->polygon().points().at(2);
            Vec2d left_up_point = target_parking_spot_ptr->polygon().points().at(3);
            Vec2d center_point = (left_bottom_point + right_bottom_point + right_up_point + left_up_point) / 4.0;
            double center_l;
            nearby_path.GetNearestPoint(center_point, &target_area_center_s, &center_l);
            break;
        }
    }
    return target_area_center_s;
}

bool StageApproachingParkingSpotPark::CheckADCInParkingRange(const Frame& frame) {
    // check stop close enough to stop line of the stop_sign
    const double adc_front_edge_s = frame.reference_line_info().front().AdcSlBoundary().end_s();
    const double stop_fence_start_s = frame.open_space_info().open_space_pre_stop_fence_s();
    const double distance_stop_line_to_adc_front_edge = stop_fence_start_s - adc_front_edge_s;
    if (distance_stop_line_to_adc_front_edge <= scenario_config_.max_valid_stop_distance()) {
        ADEBUG << "arrive stop fence or close to target.";
        return true;
    }
    return false;
}

}  // namespace planning
}  // namespace apollo
