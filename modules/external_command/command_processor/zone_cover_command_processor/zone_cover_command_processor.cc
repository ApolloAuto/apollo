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
 * @file zone_cover_command_processor.cc
 **/

#include "modules/external_command/command_processor/zone_cover_command_processor/zone_cover_command_processor.h"

namespace apollo {
namespace external_command {

bool ZoneCoverCommandProcessor::Convert(
        const std::shared_ptr<ZoneCoverCommand>& command,
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
    const auto zone_cover_area_id = command->zone_cover_area_id();
    if (zone_cover_area_id.empty()) {
        AERROR << "zone_cover_area_id is empty, error";
        return false;
    }

    hdmap::AreaInfoConstPtr zone_cover_area
            = hdmap::HDMapUtil::BaseMapPtr()->GetAreaById(hdmap::MakeMapId(zone_cover_area_id));
    if (nullptr == zone_cover_area) {
        AERROR << "can not find zone_cover_area:  " << zone_cover_area_id;
        return false;
    };

    std::vector<apollo::routing::LaneWaypoint> lane_way_points;
    lane_way_tool->GetPreciseParkingLaneWayPoint(zone_cover_area, &lane_way_points);
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

bool ZoneCoverCommandProcessor::ProcessSpecialCommand(
        const std::shared_ptr<ZoneCoverCommand>& command,
        const std::shared_ptr<apollo::planning::PlanningCommand>& planning_command) const {
    auto custom_command = planning_command->mutable_custom_command();
    custom_command->PackFrom(*command);
    return true;
}

}  // namespace external_command
}  // namespace apollo
