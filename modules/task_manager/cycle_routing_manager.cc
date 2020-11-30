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
                                   common::PointENU point_b,
                                   double distance) {
    double x_dis = point_a.x() - point_b.x();
    double y_dis = point_a.y() - point_b.y();
    return x_dis * x_dis + y_dis * y_dis < distance * distance;
}

common::Status CycleRoutingManager::Init(
        const CycleRoutingTask& cycle_routing_task) {
    cycle_ = cycle_routing_task.cycle_num();
    auto waypoints = cycle_routing_task.routing_request().waypoint();
    waypoint_num_ = waypoints.size();
    begin_point_ = waypoints[0].pose();
    end_point_ = waypoints[waypoint_num_ - 1].pose();
    is_allowed_to_route_ = true;

    AINFO << "New cycle routing task: cycle " << cycle_
        << ", begin point " << begin_point_.x() << " " << begin_point_.y()
        << ", end point " << end_point_.x() << " " << end_point_.y();

    return common::Status::OK();
}

bool CycleRoutingManager::CheckIfReachDestination(
        const localization::Pose &pose) {
    ADEBUG << "Check if reach destination: localization_pose: "
    << pose.position().x() << " " << pose.position().y()
    << ", begin point " << begin_point_.x() << " " << begin_point_.y()
    << ", end point " << end_point_.x() << " " << end_point_.y()
    << ", threshold " << FLAGS_threshold_for_destination_check
    << ", allowed_to_send_routing_request " << is_allowed_to_route_;

    if (!CheckPointDistanceInThreshold(
          begin_point_,
          pose.position(),
          FLAGS_threshold_for_destination_check * 2)) {
        is_allowed_to_route_ = true;
        return false;
    }

    if (is_allowed_to_route_ && CheckPointDistanceInThreshold(
          pose.position(),
          begin_point_,
          FLAGS_threshold_for_destination_check)) {
        is_allowed_to_route_ = false;
        return true;
    }
    return false;
}

int CycleRoutingManager::GetCycle() const {
    return cycle_;
}

void CycleRoutingManager::MinusCycle() {
    --cycle_;
}

}  // namespace task_manager
}  // namespace apollo
