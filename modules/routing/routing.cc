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

#include "modules/common/adapters/adapter_manager.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/core/navigator.h"

namespace apollo {
namespace routing {

using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::ErrorCode;

std::string Routing::Name() const { return FLAGS_node_name; }

Routing::Routing()
    : monitor_(apollo::common::monitor::MonitorMessageItem::ROUTING) {}

apollo::common::Status Routing::Init() {
  const auto routing_map_file = apollo::hdmap::RoutingMapFile();
  AINFO << "Use routing topology graph path: " << routing_map_file;
  navigator_ptr_.reset(new Navigator(routing_map_file));
  CHECK(common::util::GetProtoFromFile(FLAGS_routing_conf_file, &routing_conf_))
      << "Unable to load routing conf file: " + FLAGS_routing_conf_file;

  AINFO << "Conf file: " << FLAGS_routing_conf_file << " is loaded.";

  AdapterManager::Init(FLAGS_routing_adapter_config_filename);
  AdapterManager::AddRoutingRequestCallback(&Routing::OnRoutingRequest, this);
  return apollo::common::Status::OK();
}

apollo::common::Status Routing::Start() {
  if (!navigator_ptr_->IsReady()) {
    AERROR << "Navigator is not ready!";
    return apollo::common::Status(ErrorCode::ROUTING_ERROR,
                                  "Navigator not ready");
  }
  AINFO << "Routing service is ready.";

  apollo::common::monitor::MonitorBuffer buffer(&monitor_);
  buffer.INFO("Routing started");
  return apollo::common::Status::OK();
}

void Routing::OnRoutingRequest(
    const apollo::routing::RoutingRequest &routing_request) {
  AINFO << "Get new routing request!!!";
  routing::RoutingResponse routing_response;
  apollo::common::monitor::MonitorBuffer buffer(&monitor_);
  if (!navigator_ptr_->SearchRoute(routing_request, &routing_response)) {
    AERROR << "Failed to search route with navigator.";

    buffer.WARN("Routing failed! " + routing_response.status().msg());
    return;
  }
  buffer.INFO("Routing success!");
  AdapterManager::PublishRoutingResponse(routing_response);
  return;
}

void Routing::Stop() {}

}  // namespace routing
}  // namespace apollo
