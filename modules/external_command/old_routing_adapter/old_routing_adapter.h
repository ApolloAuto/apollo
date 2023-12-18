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
 * @file old_routing_adapter_process_component.h
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/common_msgs/external_command_msgs/lane_follow_command.pb.h"
#include "modules/common_msgs/external_command_msgs/valet_parking_command.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"

#include "cyber/component/component.h"

namespace apollo {
namespace hdmap {
class HDMap;
}  // namespace hdmap
}  // namespace apollo

/**
 * @namespace apollo::old_routing_adapter
 * @brief apollo::old_routing_adapter
 */
namespace apollo {
namespace old_routing_adapter {

/**
 * @class OldRoutingAdapter
 *
 * @brief The external interface for processing external commands.
 */
class OldRoutingAdapter : public cyber::Component<> {
 public:
  OldRoutingAdapter();

  ~OldRoutingAdapter() = default;

  bool Init() override;

 private:
  void OnRoutingRequest(
      const std::shared_ptr<apollo::routing::RoutingRequest>& routing_request);

  template <typename T>
  void CopyRoutingRequest(
      const std::shared_ptr<routing::RoutingRequest>& routing_request,
      int start_index, int end_index, T* message);

  void Convert(const apollo::routing::LaneWaypoint& lane_way_point,
               apollo::external_command::Pose* pose) const;

  double GetNearestHeading(const std::string& lane_id, double x,
                           double y) const;

  std::shared_ptr<cyber::Reader<apollo::routing::RoutingRequest>>
      routing_request_reader_;

  // Coverted from RoutingRequest without parking id.
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::LaneFollowCommand,
                            apollo::external_command::CommandStatus>>
      lane_follow_command_client_;
  // Coverted from RoutingRequest with parking id.
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::ValetParkingCommand,
                            apollo::external_command::CommandStatus>>
      valet_parking_command_client_;
  uint64_t command_id_;
  const hdmap::HDMap* hdmap_;
};

template <typename T>
void OldRoutingAdapter::CopyRoutingRequest(
    const std::shared_ptr<routing::RoutingRequest>& routing_request,
    int start_index, int end_index, T* message) {
  if (routing_request->has_header()) {
    message->mutable_header()->CopyFrom(routing_request->header());
  }
  message->set_command_id(++command_id_);
  const auto& way_points = routing_request->waypoint();
  for (int i = start_index; i < end_index; ++i) {
    Convert(way_points.Get(i), message->add_way_point());
  }
}

CYBER_REGISTER_COMPONENT(OldRoutingAdapter)

}  // namespace old_routing_adapter
}  // namespace apollo
