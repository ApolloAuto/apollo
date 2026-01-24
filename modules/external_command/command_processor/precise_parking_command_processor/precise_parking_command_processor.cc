/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
 * @file valet_parking_command_processor.cc
 **/

#include "modules/external_command/command_processor/precise_parking_command_processor/precise_parking_command_processor.h"
#include <limits>
#include <string>
#include <vector>
#include "modules/external_command/command_processor/command_processor_base/util/lane_way_tool.h"
#include "modules/map/hdmap/hdmap_common.h"

namespace apollo {
namespace external_command {

bool PreciseParkingCommandProcessor::Convert(
        const std::shared_ptr<PreciseParkingCommand>& command,
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
    std::vector<hdmap::AreaInfoConstPtr> areas;
    common::PointENU point;
    point.set_x(command->parking_spot_pose().x());
    point.set_y(command->parking_spot_pose().y());
    if (hdmap::HDMapUtil::BaseMapPtr()->GetAreas(point, 1e-1, &areas) < 0 || areas.empty()) {
        AERROR << "can not find any junction near parking spot";
        return false;
    };
    std::vector<apollo::routing::LaneWaypoint> lane_way_points;
    lane_way_tool->GetPreciseParkingLaneWayPoint(areas.front(), &lane_way_points);
    apollo::routing::LaneWaypoint best_lane_way_point;
    double min_distance = std::numeric_limits<double>::max();
    for (const auto& lane_way_point : lane_way_points) {
        auto tmp_routing_request = std::make_shared<apollo::routing::RoutingRequest>();
        tmp_routing_request->CopyFrom(*routing_request);
        tmp_routing_request->add_waypoint()->CopyFrom(lane_way_point);
        auto routing_response = std::make_shared<apollo::routing::RoutingResponse>();
        if (!routing_->Process(tmp_routing_request, routing_response.get())) {
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
        AERROR << "can not find any valid lane waypoint near parking space";
        return false;
    }
    routing_request->add_waypoint()->CopyFrom(best_lane_way_point);
    return true;
}

bool PreciseParkingCommandProcessor::ProcessSpecialCommand(
        const std::shared_ptr<PreciseParkingCommand>& command,
        const std::shared_ptr<apollo::planning::PlanningCommand>& planning_command) const {
    auto custom_command = planning_command->mutable_custom_command();
    custom_command->PackFrom(*command);
    return true;
}

}  // namespace external_command
}  // namespace apollo
