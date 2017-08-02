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

#include "modules/routing/core/navigator.h"
#include "ros/include/std_msgs/String.h"

namespace apollo {
namespace routing {

std::string Routing::Name() const { return FLAGS_node_name; }

Status Routing::Init() {
  std::string graph_path = FLAGS_graph_dir + "/" + FLAGS_graph_file_name;
  _navigator_ptr.reset(new Navigator(graph_path));
  return Status::OK();
}

Status Control::Start() {
  if (!_navigator_ptr->is_ready()) {
    AERROR << "Navigator is not ready!";
    return Status::ERROR();
  }
  ROS_INFO("Routing service is ready.");

  apollo::common::monitor::MonitorBuffer buffer(&monitor_);
  buffer.INFO("Routing started");
  return Status::OK();
}

void OnRouting_Request(const apollo::routing::RoutingRequest &routing_req) {
  AINFO << "Get new routing request!!!";
  ::apollo::routing::RoutingRequest request_proto;
  ::apollo::routing::RoutingResult response_proto;
  if (!request_proto.ParseFromString(req.routing_request.data)) {
    AERROR << "The request proto is invalid.";
    return false;
  }
  if (!_navigator_ptr->search_route(request_proto, &response_proto)) {
    AERROR<< "Failed to search route with navigator.";
    return false;
  }

  if (request_proto.broadcast()) {
    AdapterManager::PublishControlCommand(*control_command);
  }
  return true;
}

void Routing::Stop() {}

}  // namespace routing
}  // namespace apollo
