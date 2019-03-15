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

#include "modules/planning/tasks/deciders/decider_rule_based_stop.h"

#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::common::util::WithinBound;
using apollo::hdmap::MapPathPoint;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::PathOverlap;
using apollo::perception::TrafficLight;

DeciderRuleBasedStop::DeciderRuleBasedStop(const TaskConfig& config)
    : Decider(config) {
  CHECK(config.has_decider_rule_based_stop_config());
  SetName("DeciderRuleBasedStop");
}

Status DeciderRuleBasedStop::Process(Frame* frame,
                                     ReferenceLineInfo* reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  CheckStopSign(frame, reference_line_info);

  CheckTrafficLight(frame, reference_line_info);

  CheckOpenSpacePreStop(frame, reference_line_info);

  return Status::OK();
}

void DeciderRuleBasedStop::CheckStopSign(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!config_.decider_rule_based_stop_config().stop_sign().enabled()) {
    return;
  }

  const double adc_back_edge_s = reference_line_info->AdcSlBoundary().start_s();

  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info->reference_line().map_path().stop_sign_overlaps();
  for (const auto& stop_sign_overlap : stop_sign_overlaps) {
    if (stop_sign_overlap.end_s <= adc_back_edge_s) {
      continue;
    }

    const auto& stop_done_overlap_ids =
        PlanningContext::GetScenarioInfo()->stop_done_overlap_ids;
    if (stop_done_overlap_ids.end() != std::find(stop_done_overlap_ids.begin(),
                                                 stop_done_overlap_ids.end(),
                                                 stop_sign_overlap.object_id)) {
      continue;
    }

    const std::string stop_wall_id =
        STOP_SIGN_VO_ID_PREFIX + stop_sign_overlap.object_id;
    const double stop_line_s = stop_sign_overlap.start_s;
    const double stop_distance =
        config_.decider_rule_based_stop_config().stop_sign().stop_distance();
    ADEBUG << "DeciderRuleBasedStop: stop_wall_id[" << stop_wall_id
           << "] stop_line_s[" << stop_line_s << "]";

    BuildStopDecision(
        stop_wall_id, stop_line_s, stop_distance,
        StopReasonCode::STOP_REASON_STOP_SIGN,
        PlanningContext::GetScenarioInfo()->stop_sign_wait_for_obstacles, frame,
        reference_line_info);
  }
}

void DeciderRuleBasedStop::CheckTrafficLight(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!config_.decider_rule_based_stop_config().traffic_light().enabled()) {
    return;
  }

  const double adc_back_edge_s = reference_line_info->AdcSlBoundary().start_s();

  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info->reference_line().map_path().signal_overlaps();
  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    if (traffic_light_overlap.end_s <= adc_back_edge_s) {
      continue;
    }

    const auto& stop_done_overlap_ids =
        PlanningContext::GetScenarioInfo()->stop_done_overlap_ids;
    if (stop_done_overlap_ids.end() !=
        std::find(stop_done_overlap_ids.begin(), stop_done_overlap_ids.end(),
                  traffic_light_overlap.object_id)) {
      continue;
    }

    auto signal_color =
        scenario::GetSignal(traffic_light_overlap.object_id).color();
    ADEBUG << "traffic_light_id[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s << "] color["
           << signal_color << "]";
    if (signal_color == TrafficLight::GREEN) {
      continue;
    }

    // TODO(all): add stop_deceleration check based on signal colors

    const std::string stop_wall_id =
        TRAFFIC_LIGHT_VO_ID_PREFIX + traffic_light_overlap.object_id;
    const double stop_line_s = traffic_light_overlap.start_s;
    const double stop_distance = config_.decider_rule_based_stop_config()
                                     .traffic_light()
                                     .stop_distance();

    ADEBUG << "DeciderRuleBasedStop: stop_wall_id[" << stop_wall_id
           << "] stop_line_s[" << stop_line_s << "]";
    std::vector<std::string> wait_for_obstacles;
    BuildStopDecision(stop_wall_id, stop_line_s, stop_distance,
                      StopReasonCode::STOP_REASON_SIGNAL, wait_for_obstacles,
                      frame, reference_line_info);
  }
}

void DeciderRuleBasedStop::CheckOpenSpacePreStop(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  if (frame->open_space_info().open_space_pre_stop_finished()) {
    return;
  }

  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  const VehicleState& vehicle_state = frame->vehicle_state();
  const auto& target_parking_spot_id =
      frame->open_space_info().target_parking_spot_id();
  const auto& nearby_path = reference_line_info->reference_line().map_path();
  AERROR_IF(target_parking_spot_id.empty())
      << "no target parking spot id found when setting pre stop fence";

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
      double left_bottom_point_s = 0.0;
      double left_bottom_point_l = 0.0;
      double right_bottom_point_s = 0.0;
      double right_bottom_point_l = 0.0;
      nearby_path.GetNearestPoint(left_bottom_point, &left_bottom_point_s,
                                  &left_bottom_point_l);
      nearby_path.GetNearestPoint(right_bottom_point, &right_bottom_point_s,
                                  &right_bottom_point_l);
      target_area_center_s = (left_bottom_point_s + right_bottom_point_s) / 2.0;
      target_area_found = true;
    }
  }
  AERROR_IF(!target_area_found)
      << "no target parking spot found on reference line";

  double stop_line_s = 0.0;
  double stop_distance_to_target = config_.decider_rule_based_stop_config()
                                       .open_space()
                                       .stop_distance_to_target();
  double static_linear_velocity_epsilon = 1.0e-2;
  CHECK_GE(stop_distance_to_target, 1.0e-8);
  double target_vehicle_offset = target_area_center_s - adc_front_edge_s;
  if (target_vehicle_offset > stop_distance_to_target) {
    stop_line_s = target_area_center_s - stop_distance_to_target;
  } else if (std::abs(target_vehicle_offset) < stop_distance_to_target) {
    stop_line_s = target_area_center_s + stop_distance_to_target;
  } else if (target_vehicle_offset < -stop_distance_to_target) {
    if (!frame->open_space_info().pre_stop_rightaway_flag()) {
      // TODO(Jinyun) Use constant comfortable deacceleration rather than
      // distance by config to set stop fence
      stop_line_s = adc_front_edge_s + config_.decider_rule_based_stop_config()
                                           .open_space()
                                           .rightaway_stop_distance();
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

  const std::string stop_wall_id =
      OPEN_SPACE_VO_ID_PREFIX + target_parking_spot_id;
  std::vector<std::string> wait_for_obstacles;
  frame->mutable_open_space_info()->set_open_space_pre_stop_fence_s(
      stop_line_s);
  BuildStopDecision(stop_wall_id, stop_line_s, 0.0,
                    StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                    wait_for_obstacles, frame, reference_line_info);
}

// TODO(Jinyun) Move to more general folder for common use
bool DeciderRuleBasedStop::BuildStopDecision(
    const std::string& stop_wall_id, const double stop_line_s,
    const double stop_distance, const StopReasonCode& stop_reason_code,
    const std::vector<std::string>& wait_for_obstacles, Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // check
  const auto& reference_line = reference_line_info->reference_line();
  if (!WithinBound(0.0, reference_line.Length(), stop_line_s)) {
    AERROR << "stop_line_s[" << stop_line_s << "] is not on reference line";
    return 0;
  }

  // create virtual stop wall
  auto* obstacle =
      frame->CreateStopObstacle(reference_line_info, stop_wall_id, stop_line_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
    return -1;
  }
  Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create obstacle for: " << stop_wall_id;
    return -1;
  }

  // build stop decision
  const double stop_s = stop_line_s - stop_distance;
  auto stop_point = reference_line.GetReferencePoint(stop_s);
  double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(stop_reason_code);
  stop_decision->set_distance_s(-stop_distance);
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  for (size_t i = 0; i < wait_for_obstacles.size(); ++i) {
    stop_decision->add_wait_for_obstacle(wait_for_obstacles[i]);
  }

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision("DeciderRuleBasedStop",
                                         stop_wall->Id(), stop);
  return 0;
}

}  // namespace planning
}  // namespace apollo
