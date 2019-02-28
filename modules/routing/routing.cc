/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/routing/routing.h"

#include "modules/common/util/util.h"
#include "modules/routing/common/routing_gflags.h"

namespace apollo {
namespace routing {

using apollo::common::ErrorCode;

std::string Routing::Name() const { return FLAGS_routing_node_name; }

Routing::Routing()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::ROUTING) {}

apollo::common::Status Routing::Init() {
  const auto routing_map_file = apollo::hdmap::RoutingMapFile();
  AINFO << "Use routing topology graph path: " << routing_map_file;
  navigator_ptr_.reset(new Navigator(routing_map_file));
  CHECK(
      cyber::common::GetProtoFromFile(FLAGS_routing_conf_file, &routing_conf_))
      << "Unable to load routing conf file: " + FLAGS_routing_conf_file;

  AINFO << "Conf file: " << FLAGS_routing_conf_file << " is loaded.";

  hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
  CHECK(hdmap_) << "Failed to load map file:" << apollo::hdmap::BaseMapFile();

  return apollo::common::Status::OK();
}

apollo::common::Status Routing::Start() {
  if (!navigator_ptr_->IsReady()) {
    AERROR << "Navigator is not ready!";
    return apollo::common::Status(ErrorCode::ROUTING_ERROR,
                                  "Navigator not ready");
  }
  AINFO << "Routing service is ready.";
  monitor_logger_buffer_.INFO("Routing started");
  return apollo::common::Status::OK();
}

RoutingRequest Routing::FillLaneInfoIfMissing(
    const RoutingRequest& routing_request) {
  RoutingRequest fixed_request(routing_request);
  for (int i = 0; i < routing_request.waypoint_size(); ++i) {
    const auto& lane_waypoint = routing_request.waypoint(i);
    if (lane_waypoint.has_id()) {
      continue;
    }
    auto point = common::util::MakePointENU(lane_waypoint.pose().x(),
                                            lane_waypoint.pose().y(),
                                            lane_waypoint.pose().z());

    double s = 0.0;
    double l = 0.0;
    hdmap::LaneInfoConstPtr lane;
    // FIXME(all): select one reasonable lane candidate for point=>lane
    // is one to many relationship.
    if (hdmap_->GetNearestLane(point, &lane, &s, &l) != 0) {
      AERROR << "Failed to find nearest lane from map at position: "
             << point.DebugString();
      return routing_request;
    }
    auto waypoint_info = fixed_request.mutable_waypoint(i);
    waypoint_info->set_id(lane->id().id());
    waypoint_info->set_s(s);
  }
  AINFO << "Fixed routing request:" << fixed_request.DebugString();
  return fixed_request;
}

bool Routing::Process(const std::shared_ptr<RoutingRequest>& routing_request,
                      RoutingResponse* const routing_response) {
  CHECK_NOTNULL(routing_response);
  AINFO << "Get new routing request:" << routing_request->DebugString();
  const auto& fixed_request = FillLaneInfoIfMissing(*routing_request);
  if (!navigator_ptr_->SearchRoute(fixed_request, routing_response)) {
    AERROR << "Failed to search route with navigator.";

    monitor_logger_buffer_.WARN("Routing failed! " +
                                routing_response->status().msg());
    return false;
  }
  monitor_logger_buffer_.INFO("Routing success!");
  return true;
}

}  // namespace routing
}  // namespace apollo
