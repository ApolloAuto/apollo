/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/task_manager/parking_routing_manager.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/task_manager/common/task_manager_gflags.h"

namespace apollo {
namespace task_manager {

using apollo::common::PointENU;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::routing::ParkingSpaceType;

ParkingRoutingManager::ParkingRoutingManager()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::TASK_MANAGER) {}

common::Status ParkingRoutingManager::Init(
    const ParkingRoutingTask& parking_routing_task) {
  return common::Status::OK();
}

bool ParkingRoutingManager::ConstructParkingRoutingRequest(
    ParkingRoutingTask* parking_routing_task) {
  auto hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  hdmap::Id id;
  id.set_id(parking_routing_task->routing_request()
                .parking_info()
                .parking_space_id());
  auto parking_space_info = hdmap_->GetParkingSpaceById(id);
  auto request_parking_info =
      parking_routing_task->mutable_routing_request()->mutable_parking_info();
  if (parking_space_info == nullptr) {
    AERROR << "Can not find parking space" << id_ << "in map";
    return false;
  }
  auto points = parking_space_info->polygon().points();
  // 0 1 2 3: left_top right_top right_rear left_rear corner point for heading
  // upward
  Vec2d center_point(0, 0);
  for (size_t i = 0; i < points.size(); i++) {
    center_point += points[i];
  }
  center_point /= 4.0;
  request_parking_info->mutable_parking_point()->set_x(center_point.x());
  request_parking_info->mutable_parking_point()->set_y(center_point.y());
  request_parking_info->mutable_parking_point()->set_z(0);
  apollo::common::PointENU center_enu;
  center_enu.set_x(center_point.x());
  center_enu.set_y(center_point.y());
  apollo::hdmap::LaneInfoConstPtr nearest_lane;
  double nearest_s;
  double nearest_l;
  hdmap_->GetNearestLane(center_enu, &nearest_lane, &nearest_s, &nearest_l);
  double lane_heading = nearest_lane->Heading(nearest_s);
  double diff_angle = common::math::AngleDiff(
      lane_heading, parking_space_info->parking_space().heading());

  if (std::fabs(diff_angle) < M_PI / 3.0) {
    AINFO << "Find a parallel parking" << id_ << "lane_heading" << lane_heading
          << "parking heading" << parking_space_info->parking_space().heading();
    request_parking_info->set_parking_space_type(
        ParkingSpaceType::PARALLEL_PARKING);
    auto left_down = request_parking_info->mutable_corner_point()->add_point();
    left_down->set_x(points[0].x());
    left_down->set_y(points[0].y());
    auto right_down = request_parking_info->mutable_corner_point()->add_point();
    right_down->set_x(points[1].x());
    right_down->set_y(points[1].y());
    auto right_up = request_parking_info->mutable_corner_point()->add_point();
    right_up->set_x(points[2].x());
    right_up->set_y(points[2].y());
    auto left_up = request_parking_info->mutable_corner_point()->add_point();
    left_up->set_x(points[3].x());
    left_up->set_y(points[3].y());
  } else {
    AINFO << "Find a vertical parking";
    request_parking_info->set_parking_space_type(
        ParkingSpaceType::VERTICAL_PLOT);
    auto left_down = request_parking_info->mutable_corner_point()->add_point();
    left_down->set_x(points[0].x());
    left_down->set_y(points[0].y());
    auto right_down = request_parking_info->mutable_corner_point()->add_point();
    right_down->set_x(points[1].x());
    right_down->set_y(points[1].y());
    auto right_up = request_parking_info->mutable_corner_point()->add_point();
    right_up->set_x(points[2].x());
    right_up->set_y(points[2].y());
    auto left_up = request_parking_info->mutable_corner_point()->add_point();
    left_up->set_x(points[3].x());
    left_up->set_y(points[3].y());
  }
  // extend last point to aviod referenceline generated failed in parking
  auto last_waypoint = parking_routing_task->mutable_routing_request()
                           ->mutable_waypoint()
                           ->rbegin();
  auto extend_waypoint = parking_routing_task->mutable_routing_request()
                           ->mutable_waypoint()
                           ->Add();
  static constexpr double kExtendParkingLength = 20;
  apollo::common::PointENU extend_point;
  extend_point.set_x(last_waypoint->pose().x() +
                     kExtendParkingLength * std::cos(lane_heading));
  extend_point.set_y(last_waypoint->pose().y() +
                     kExtendParkingLength * std::sin(lane_heading));
  hdmap_->GetNearestLaneWithHeading(extend_point, 20, lane_heading, M_PI_2,
                                    &nearest_lane, &nearest_s, &nearest_l);
  extend_point = nearest_lane->GetSmoothPoint(nearest_s);
  extend_waypoint->mutable_pose()->set_x(extend_point.x());
  extend_waypoint->mutable_pose()->set_y(extend_point.y());
  extend_waypoint->set_id(nearest_lane->id().id());
  extend_waypoint->set_s(nearest_s);

  return true;
}

}  // namespace task_manager
}  // namespace apollo
