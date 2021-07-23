/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include "modules/task_manager/dead_end_routing_manager.h"
#include "modules/task_manager/common/task_manager_gflags.h"

namespace apollo {
namespace task_manager {
using apollo::common::VehicleState;
using apollo::common::PointENU;
using apollo::routing::RoutingRequest;
using apollo::hdmap::JunctionInfoConstPtr;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

DeadEndRoutingManager::DeadEndRoutingManager()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::TASK_MANAGER) {}

common::Status DeadEndRoutingManager::Init(
    const DeadEndRoutingTask& dead_end_routing_task) {
  cycle_ = 2;
  routing_request_in_ = dead_end_routing_task.routing_request_in();
  routing_request_out_ = dead_end_routing_task.routing_request_out();
  return common::Status::OK();
}

bool DeadEndRoutingManager::JudgeCarInDeadEndJunction(
    const Vec2d& car_position,
    const common::PointENU& target_point) {
  size_t waypoint_num = routing_request_in_.waypoint().size();
  PointENU dead_end_point = routing_request_in_
                            .waypoint().at(waypoint_num - 1).pose();
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  std::vector<JunctionInfoConstPtr> junctions;
  JunctionInfoConstPtr junction;
  if (base_map_ptr->GetJunctions(
      dead_end_point, FLAGS_search_junction_threshold, &junctions) != 0) {
    AERROR << "Fail to get junctions from base_map.";
    return false;
  }
  if (junctions.size() <= 0) {
    AERROR << "No junction from map";
    return false;
  }
  size_t junction_num = junctions.size();
  for (size_t i = 0; i < junction_num; ++i) {
    if (junctions.at(i)->junction().type() == DEAD_END) {
      double distance_to_vehicle =
        (car_position.x() - target_point.x()) *
        (car_position.x() - target_point.x()) +
        (car_position.y() - target_point.y()) *
        (car_position.y() - target_point.y());
      if (distance_to_vehicle < FLAGS_dead_end_destination_threshold) {
        return true;
      }
    } else {
      return false;
    }
  }
  return false;
}

bool DeadEndRoutingManager::GetNewRouting(
  const localization::Pose& pose,
  routing::RoutingRequest* routing_request) {
  Vec2d car_position;
  car_position.set_x(pose.position().x());
  car_position.set_y(pose.position().y());
  const common::PointENU& target_point =
    routing_request_out_.waypoint().at(0).pose();
  if (JudgeCarInDeadEndJunction(car_position, target_point)) {
    if (routing_out_flag_) {
      *routing_request = routing_request_out_;
      routing_request->mutable_dead_end_info()->
        set_dead_end_routing_type(routing::ROUTING_OUT);
      --cycle_;
      routing_out_flag_ = false;
      return true;
    }
  } else {
    if (routing_in_flag_) {
      *routing_request = routing_request_in_;
      routing_request->mutable_dead_end_info()->
        set_dead_end_routing_type(routing::ROUTING_IN);
      routing_request->mutable_dead_end_info()->
        mutable_target_point()->CopyFrom(target_point);
      routing_in_flag_ = false;
      return true;
    }
  }
  return false;
}

}  // namespace task_manager
}  // namespace apollo
