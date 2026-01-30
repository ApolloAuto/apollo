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

using common::math::Vec2d;

bool CheckPointDistanceInThreshold(external_command::Pose target,
                                   common::PointENU ego, double distance) {
  AINFO << ego.x() << " " << ego.y() << " " << target.x() << " " << target.y();
  Vec2d tar2ego(ego.x() - target.x(), ego.y() - target.y());
  Vec2d unit = Vec2d::CreateUnitVec2d(target.heading());
  double dis = unit.InnerProd(tar2ego);
  return dis < 1e-6 && std::fabs(dis) < distance && tar2ego.Length() < distance;
}

common::Status CycleRoutingManager::Init(
    const localization::Pose& pose,
    const CycleRoutingTask& cycle_routing_task) {
  cycle_ = cycle_routing_task.cycle_num();
  auto waypoints = cycle_routing_task.lane_follow_command().way_point();
  begin_point_.set_x(pose.position().x());
  begin_point_.set_y(pose.position().y());
  begin_point_.set_heading(pose.heading());
  end_point_ = cycle_routing_task.lane_follow_command().end_pose();
  is_allowed_to_route_ = true;
  original_lane_follow_command_ = cycle_routing_task.lane_follow_command();
  map_service_.reset(new apollo::dreamview::MapService());

  AINFO << "New cycle routing task: cycle " << cycle_ << ", begin point "
        << begin_point_.x() << " " << begin_point_.y() << ", end point "
        << end_point_.x() << " " << end_point_.y();

  return common::Status::OK();
}

bool CycleRoutingManager::GetNewRouting(
    const localization::Pose& pose,
    external_command::LaneFollowCommand* lane_follow_command) {
  AINFO << "GetNewRouting: localization_pose: " << pose.position().x() << " "
        << pose.position().y() << ", begin point " << begin_point_.x() << " "
        << begin_point_.y() << ", end point " << end_point_.x() << " "
        << end_point_.y() << ", threshold "
        << FLAGS_task_manager_threshold_for_destination_check
        << ", allowed_to_send_routing_request " << is_allowed_to_route_;

  if (is_allowed_to_route_) {
    if (CheckPointDistanceInThreshold(
            begin_point_, pose.position(),
            FLAGS_task_manager_threshold_for_destination_check)) {
      AINFO << "GetNewRouting: reach begin point."
            << "Remaining cycles: " << cycle_;
      lane_follow_command->CopyFrom(original_lane_follow_command_);
      auto cur_point = lane_follow_command->mutable_way_point(0);
      is_allowed_to_route_ = false;
      return true;
    }
  } else {
    if (CheckPointDistanceInThreshold(
            end_point_, pose.position(),
            FLAGS_task_manager_threshold_for_destination_check)) {
      AINFO << "GetNewRouting: reach end point. "
            << "Remaining cycles: " << cycle_;
      lane_follow_command->clear_way_point();
      auto cur_point = lane_follow_command->add_way_point();
      cur_point->Clear();
      cur_point->set_x(pose.position().x());
      cur_point->set_y(pose.position().y());
      cur_point->set_heading(pose.heading());
      auto end_pose = lane_follow_command->mutable_end_pose();
      end_pose->CopyFrom(begin_point_);
      --cycle_;
      is_allowed_to_route_ = true;
      return true;
    }
  }

  return false;
}

}  // namespace task_manager
}  // namespace apollo
