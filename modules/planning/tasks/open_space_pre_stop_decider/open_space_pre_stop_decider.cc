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

#include "modules/planning/tasks/open_space_pre_stop_decider/open_space_pre_stop_decider.h"

#include <memory>
#include <string>
#include <vector>

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;

bool OpenSpacePreStopDecider::Init(
    const std::string& config_dir, const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (!Decider::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  bool res = Decider::LoadConfig<OpenSpacePreStopDeciderConfig>(&config_);
  AINFO << "Load config:" << config_.DebugString();
  return res;
}

Status OpenSpacePreStopDecider::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  double target_s = 0.0;
  const auto& stop_type = config_.stop_type();
  switch (stop_type) {
    case OpenSpacePreStopDeciderConfig::PARKING:
      if (!CheckParkingSpotPreStop(frame, reference_line_info, &target_s)) {
        const std::string msg = "Checking parking spot pre stop fails";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      SetParkingSpotStopFence(target_s, frame, reference_line_info);
      break;
    case OpenSpacePreStopDeciderConfig::PULL_OVER:
      if (!CheckPullOverPreStop(frame, reference_line_info, &target_s)) {
        const std::string msg = "Checking pull over pre stop fails";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      SetPullOverStopFence(target_s, frame, reference_line_info);
      break;
    default:
      const std::string msg = "This stop type not implemented";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

bool OpenSpacePreStopDecider::CheckPullOverPreStop(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    double* target_s) {
  *target_s = 0.0;
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  if (pull_over_status.has_position() && pull_over_status.position().has_x() &&
      pull_over_status.position().has_y()) {
    common::SLPoint pull_over_sl;
    const auto& reference_line = reference_line_info->reference_line();
    reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
    *target_s = pull_over_sl.s();
  }
  return true;
}

bool OpenSpacePreStopDecider::CheckParkingSpotPreStop(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    double* target_s) {
  const auto& target_parking_spot_id =
      frame->open_space_info().target_parking_spot_id();
  const auto& nearby_path = reference_line_info->reference_line().map_path();
  if (target_parking_spot_id.empty()) {
    AERROR << "no target parking spot id found when setting pre stop fence";
    return false;
  }

  double target_area_center_s = 0.0;
  bool target_area_found = false;
  const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
  ParkingSpaceInfoConstPtr target_parking_spot_ptr;
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  for (const auto& parking_overlap : parking_space_overlaps) {
    if (parking_overlap.object_id == target_parking_spot_id) {
      // TODO(Jinyun) parking overlap s are wrong on map, not usable
      // target_area_center_s =
      //     (parking_overlap.start_s + parking_overlap.end_s) / 2.0;
      hdmap::Id id;
      id.set_id(parking_overlap.object_id);
      target_parking_spot_ptr = hdmap->GetParkingSpaceById(id);
      Vec2d left_bottom_point =
          target_parking_spot_ptr->polygon().points().at(0);
      Vec2d right_bottom_point =
          target_parking_spot_ptr->polygon().points().at(1);
      Vec2d right_up_point = target_parking_spot_ptr->polygon().points().at(2);
      Vec2d left_up_point = target_parking_spot_ptr->polygon().points().at(3);
      Vec2d center_point = (left_bottom_point + right_bottom_point +
                            right_up_point + left_up_point) /
                           4.0;
      double center_l;
      nearby_path.GetNearestPoint(center_point, &target_area_center_s,
                                  &center_l);
      target_area_found = true;
    }
  }

  if (!target_area_found) {
    AERROR << "no target parking spot found on reference line";
    return false;
  }
  *target_s = target_area_center_s;
  return true;
}

void OpenSpacePreStopDecider::SetParkingSpotStopFence(
    const double target_s, Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  const auto& nearby_path = reference_line_info->reference_line().map_path();
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  const double front_edge_to_center = common::VehicleConfigHelper::Instance()
                                          ->GetConfig()
                                          .vehicle_param()
                                          .front_edge_to_center();
  double ego_s = adc_front_edge_s - front_edge_to_center;
  const VehicleState& vehicle_state = frame->vehicle_state();
  double stop_line_s = 0.0;
  double stop_distance_to_target = config_.stop_distance_to_target();
  double static_linear_velocity_epsilon = 1.0e-2;
  static constexpr double kStopBuffer = 0.2;
  CHECK_GE(stop_distance_to_target, 1.0e-8);
  stop_line_s =
      target_s + front_edge_to_center + config_.stop_buffer_to_target();
  const std::string stop_wall_id = OPEN_SPACE_STOP_ID;
  std::vector<std::string> wait_for_obstacles;
  frame->mutable_open_space_info()->set_open_space_pre_stop_fence_s(
      stop_line_s);
  util::BuildStopDecision(stop_wall_id, stop_line_s, 0.0,
                          StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                          wait_for_obstacles, "OpenSpacePreStopDecider", frame,
                          reference_line_info);
}

void OpenSpacePreStopDecider::SetPullOverStopFence(
    const double target_s, Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  const auto& nearby_path = reference_line_info->reference_line().map_path();
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  const VehicleState& vehicle_state = frame->vehicle_state();
  double stop_line_s = 0.0;
  double stop_distance_to_target = config_.stop_distance_to_target();
  double static_linear_velocity_epsilon = 1.0e-2;
  CHECK_GE(stop_distance_to_target, 1.0e-8);
  double target_vehicle_offset = target_s - adc_front_edge_s;
  if (target_vehicle_offset > stop_distance_to_target) {
    stop_line_s = target_s - stop_distance_to_target;
  } else {
    if (!frame->open_space_info().pre_stop_rightaway_flag()) {
      // TODO(Jinyun) Use constant comfortable deacceleration rather than
      // distance by config to set stop fence
      stop_line_s = adc_front_edge_s + config_.rightaway_stop_distance();
      if (std::abs(vehicle_state.linear_velocity()) <
          static_linear_velocity_epsilon) {
        stop_line_s = adc_front_edge_s;
      }
      *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point()) =
          nearby_path.GetSmoothPoint(stop_line_s);
      frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(true);
    } else {
      double stop_point_s = 0.0;
      double stop_point_l = 0.0;
      nearby_path.GetNearestPoint(
          frame->open_space_info().pre_stop_rightaway_point(), &stop_point_s,
          &stop_point_l);
      stop_line_s = stop_point_s;
    }
  }

  const std::string stop_wall_id = OPEN_SPACE_STOP_ID;
  std::vector<std::string> wait_for_obstacles;
  frame->mutable_open_space_info()->set_open_space_pre_stop_fence_s(
      stop_line_s);
  util::BuildStopDecision(stop_wall_id, stop_line_s, 0.0,
                          StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                          wait_for_obstacles, "OpenSpacePreStopDecider", frame,
                          reference_line_info);
}
}  // namespace planning
}  // namespace apollo
