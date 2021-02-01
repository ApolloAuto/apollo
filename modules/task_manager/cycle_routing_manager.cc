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
#include "modules/task_manager/cycle_routing_manager.h"

#include "modules/task_manager/common/task_manager_gflags.h"

namespace apollo {
namespace task_manager {
bool CheckPointDistanceInThreshold(common::PointENU point_a,
                                   common::PointENU point_b, double distance) {
  double x_dis = point_a.x() - point_b.x();
  double y_dis = point_a.y() - point_b.y();
  return x_dis * x_dis + y_dis * y_dis < distance * distance;
}

common::Status CycleRoutingManager::Init(
    const CycleRoutingTask& cycle_routing_task) {
  cycle_ = cycle_routing_task.cycle_num();
  auto waypoints = cycle_routing_task.routing_request().waypoint();
  waypoint_num_ = waypoints.size();
  begin_point_ = waypoints[0];
  end_point_ = waypoints[waypoint_num_ - 1];
  is_allowed_to_route_ = true;
  original_routing_request_ = cycle_routing_task.routing_request();
  map_service_.reset(new apollo::dreamview::MapService());

  AINFO << "New cycle routing task: cycle " << cycle_ << ", begin point "
        << begin_point_.pose().x() << " " << begin_point_.pose().y()
        << ", end point " << end_point_.pose().x() << " "
        << end_point_.pose().y();

  return common::Status::OK();
}

bool CycleRoutingManager::GetNewRouting(
    const localization::Pose& pose,
    routing::RoutingRequest* new_routing_request) {
  AINFO << "GetNewRouting: localization_pose: " << pose.position().x() << " "
        << pose.position().y() << ", begin point " << begin_point_.pose().x()
        << " " << begin_point_.pose().y() << ", end point "
        << end_point_.pose().x() << " " << end_point_.pose().y()
        << ", threshold " << FLAGS_threshold_for_destination_check
        << ", allowed_to_send_routing_request " << is_allowed_to_route_;

  if (is_allowed_to_route_) {
    if (CheckPointDistanceInThreshold(begin_point_.pose(), pose.position(),
                                      FLAGS_threshold_for_destination_check)) {
      AINFO << "GetNewRouting: reach begin point";
      new_routing_request->CopyFrom(original_routing_request_);
      auto cur_point = new_routing_request->mutable_waypoint(0);
      if (!map_service_->ConstructLaneWayPointWithHeading(
              pose.position().x(), pose.position().y(), pose.heading(),
              cur_point)) {
        AINFO << "GetNewRouting: construct begin lane way point fail!";
        return false;
      }
      --cycle_;
      is_allowed_to_route_ = false;
      return true;
    }
  } else {
    if (CheckPointDistanceInThreshold(end_point_.pose(), pose.position(),
                                      FLAGS_threshold_for_destination_check)) {
      AINFO << "GetNewRouting: reach end point";
      new_routing_request->clear_waypoint();
      auto cur_point = new_routing_request->add_waypoint();
      if (!map_service_->ConstructLaneWayPointWithHeading(
              pose.position().x(), pose.position().y(), pose.heading(),
              cur_point)) {
        AINFO << "GetNewRouting: construct end lane way point fail!";
        return false;
      }
      auto next_point = new_routing_request->add_waypoint();
      next_point->CopyFrom(begin_point_);
      is_allowed_to_route_ = true;
      return true;
    }
  }

  return false;
}

}  // namespace task_manager
}  // namespace apollo
