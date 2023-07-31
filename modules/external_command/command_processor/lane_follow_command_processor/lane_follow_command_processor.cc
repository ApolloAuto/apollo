/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file lane_follow_command_processor.cc
 **/

#include "modules/external_command/command_processor/lane_follow_command_processor/lane_follow_command_processor.h"
#include "modules/external_command/command_processor/command_processor_base/util/lane_way_tool.h"

namespace apollo {
namespace external_command {

bool LaneFollowCommandProcessor::ProcessSpecialCommand(
    const std::shared_ptr<LaneFollowCommand>& command,
    const std::shared_ptr<apollo::planning::PlanningCommand>& planning_command)
    const {
  return true;
}

bool LaneFollowCommandProcessor::Convert(
    const std::shared_ptr<LaneFollowCommand>& command,
    std::shared_ptr<apollo::routing::RoutingRequest>& routing_request) const {
  routing_request = std::make_shared<apollo::routing::RoutingRequest>();
  if (!command->is_start_pose_set()) {
    if (!SetStartPose(routing_request)) {
      return false;
    }
  } else {
    routing_request->set_is_start_pose_set(true);
  }
  auto lane_way_tool = GetLaneWayTool();
  // Add the way points into RoutingRequest.
  for (const auto& way_point : command->way_point()) {
    if (!lane_way_tool->ConvertToLaneWayPoint(
            way_point, routing_request->add_waypoint())) {
      AERROR << "Cannot convert the end pose to lane way point: "
             << way_point.DebugString();
      return false;
    }
  }
  // Get the end pose and update in RoutingRequest.
  if (!lane_way_tool->ConvertToLaneWayPoint(command->end_pose(),
                                            routing_request->add_waypoint())) {
    AERROR << "Cannot convert the end pose to lane way point: "
           << command->end_pose().DebugString();
    return false;
  }
  return true;
}

}  // namespace external_command
}  // namespace apollo
