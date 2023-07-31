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
 * @file old_routing_adapter.cc
 */

#include "modules/external_command/old_routing_adapter/old_routing_adapter.h"

#include <limits>

#include "modules/external_command/old_routing_adapter/proto/old_routing_adapter.pb.h"

#include "modules/map/hdmap/hdmap_util.h"

/**
 * @namespace apollo::old_routing_adapter
 * @brief apollo::old_routing_adapter
 */
namespace apollo {
namespace old_routing_adapter {

using apollo::external_command::LaneFollowCommand;
using apollo::external_command::ValetParkingCommand;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;

OldRoutingAdapter::OldRoutingAdapter() : command_id_(0), hdmap_(nullptr) {}

bool OldRoutingAdapter::Init() {
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  // Load the config.
  OldRoutingAdapterConfig config;
  if (!GetProtoConfig(&config)) {
    AERROR << "Unable to load OldRoutingAdapter conf file: "
           << ConfigFilePath();
    return false;
  }
  // Init the readers, writers and service clients.
  routing_request_reader_ = node_->CreateReader<routing::RoutingRequest>(
      config.routing_request_topic(),
      [this](const std::shared_ptr<routing::RoutingRequest>& routing_request) {
        ADEBUG << "Received routing request: run callback.";
        OnRoutingRequest(routing_request);
      });
  lane_follow_command_client_ =
      node_->CreateClient<apollo::external_command::LaneFollowCommand,
                          apollo::external_command::CommandStatus>(
          config.lane_follow_command_topic());
  valet_parking_command_client_ =
      node_->CreateClient<apollo::external_command::ValetParkingCommand,
                          apollo::external_command::CommandStatus>(
          config.valet_parking_command_topic());
  return true;
}

void OldRoutingAdapter::OnRoutingRequest(
    const std::shared_ptr<routing::RoutingRequest>& routing_request) {
  // Delay sending routing request and wait for localization updated.
  cyber::SleepFor(std::chrono::microseconds(500));
  AINFO << "Receive routing request " << routing_request->DebugString();
  if (routing_request->has_parking_info()) {
    auto valet_parking_command = std::make_shared<ValetParkingCommand>();
    // Copy the way points from RoutingRequest.
    // int way_point_size = routing_request->waypoint().size();
    // if (way_point_size > 0) {
    //   CopyRoutingRequest<ValetParkingCommand>(
    //       routing_request, 0, routing_request->waypoint().size(),
    //       valet_parking_command.get());
    // }
    valet_parking_command->set_parking_spot_id(
        routing_request->parking_info().parking_space_id());
    auto response =
        valet_parking_command_client_->SendRequest(valet_parking_command);
    AINFO << "Send valet parking command "
          << valet_parking_command->DebugString();
    if (nullptr == response) {
      AERROR << "Failed to send request of valet parking!";
    }
  } else {
    auto lane_follow_command = std::make_shared<LaneFollowCommand>();
    // Copy the way points from RoutingRequest.
    int way_point_size = routing_request->waypoint().size();
    AINFO << routing_request->DebugString();
    CHECK_GT(way_point_size, 0);
    if (way_point_size > 1) {
      routing_request->mutable_waypoint()->DeleteSubrange(0, 1);
      CopyRoutingRequest<LaneFollowCommand>(
          routing_request, 0, way_point_size - 2, lane_follow_command.get());
      --way_point_size;
    }
    // Copy the end point.
    lane_follow_command->mutable_header()->CopyFrom(routing_request->header());
    Convert(routing_request->waypoint().Get(way_point_size - 1),
            lane_follow_command->mutable_end_pose());
    AINFO << "Send lane follow command " << lane_follow_command->DebugString();
    auto response =
        lane_follow_command_client_->SendRequest(lane_follow_command);
    if (nullptr == response) {
      AERROR << "Failed to get response of lane follow!";
    }
  }
}

void OldRoutingAdapter::Convert(
    const apollo::routing::LaneWaypoint& lane_way_point,
    apollo::external_command::Pose* pose) const {
  CHECK(lane_way_point.has_pose());
  const auto& way_pose = lane_way_point.pose();
  pose->set_x(way_pose.x());
  pose->set_y(way_pose.y());
  if (lane_way_point.has_heading()) {
    pose->set_heading(lane_way_point.heading());
    return;
  }
  // Get heading according to lane id
  if (lane_way_point.has_id()) {
    pose->set_heading(
        GetNearestHeading(lane_way_point.id(), way_pose.x(), way_pose.y()));
  }
}

double OldRoutingAdapter::GetNearestHeading(const std::string& lane_id,
                                            double x, double y) const {
  hdmap::Id id;
  id.set_id(lane_id);
  auto lane = hdmap_->GetLaneById(id);
  AINFO << "hdmap lane id " << lane_id << " " << hdmap::BaseMapFile();
  CHECK_NOTNULL(lane);
  common::math::Vec2d proj_pt(0.0, 0.0);
  common::math::Vec2d point(x, y);
  double s_offset = 0.0;
  int s_offset_index = 0;
  lane->DistanceTo(point, &proj_pt, &s_offset, &s_offset_index);
  return lane->headings()[s_offset_index];
}

}  // namespace old_routing_adapter
}  // namespace apollo
