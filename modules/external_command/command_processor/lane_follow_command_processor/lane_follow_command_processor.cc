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

#include <limits>
#include <vector>

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
  std::vector<apollo::routing::LaneWaypoint> lane_way_points;
  auto lane_way_tool = GetLaneWayTool();
  if (!command->is_start_pose_set()) {
    if (lane_way_tool->IsParkandgoScenario()) {
      AINFO << "adc is outside road, in park and go scenario";
      if (!SetStartPose(&lane_way_points)) {
        return false;
      }
      apollo::routing::LaneWaypoint best_lane_way_point;
      double min_distance = std::numeric_limits<double>::max();
      AINFO << "lane_way_points.size()" << lane_way_points.size();
      for (const auto& lane_way_point : lane_way_points) {
        auto tmp_routing_request =
            std::make_shared<apollo::routing::RoutingRequest>();
        tmp_routing_request->CopyFrom(*routing_request);
        tmp_routing_request->add_waypoint()->CopyFrom(lane_way_point);
        for (const auto& way_point : command->way_point()) {
          if (!lane_way_tool->ConvertToLaneWayPoint(
                  way_point, tmp_routing_request->add_waypoint())) {
            AINFO << "Cannot convert the end pose to lane way point: "
                  << way_point.DebugString();
            return false;
          }
        }
        if (!lane_way_tool->ConvertToLaneWayPoint(
                command->end_pose(), tmp_routing_request->add_waypoint())) {
          AERROR << "Cannot convert the end pose to lane way point: "
                 << command->end_pose().DebugString();
          return false;
        }
        auto routing_response =
            std::make_shared<apollo::routing::RoutingResponse>();
        if (!routing_->Process(tmp_routing_request, routing_response.get())) {
          AINFO << "routing_ error  " << tmp_routing_request->DebugString();
          continue;
        }
        AINFO << routing_response->DebugString();
        if (routing_response->measurement().distance() < min_distance) {
          min_distance = routing_response->measurement().distance();
          best_lane_way_point.Clear();
          best_lane_way_point.CopyFrom(lane_way_point);
        }
      }
      if (min_distance >= std::numeric_limits<double>::max()) {
        AINFO << "can not find any valid lane waypoint near parking space";
        return false;
      }
      routing_request->add_waypoint()->CopyFrom(best_lane_way_point);
    } else {
      if (!SetStartPose(routing_request)) {
        return false;
      }
    }
  } else {
    routing_request->set_is_start_pose_set(true);
  }
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
