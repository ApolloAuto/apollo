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

#include "modules/dreamview/backend/dreamview.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/time/time.h"

#include "modules/dreamview/backend/common/dreamview_gflags.h"

namespace apollo {
namespace dreamview {

using apollo::common::adapter::AdapterManager;
using apollo::common::VehicleConfigHelper;
using apollo::common::Status;
using apollo::common::time::Clock;

std::string Dreamview::Name() const {
  return FLAGS_dreamview_module_name;
}

Status Dreamview::Init() {
  AdapterManager::Init();
  VehicleConfigHelper::Init();

  // Initialize and run the web server which serves the dreamview htmls and
  // javascripts and handles websocket requests.
  server_.reset(
      new CivetServer({"document_root", FLAGS_static_file_dir,
                       "listening_ports", std::to_string(FLAGS_server_port)}));
  websocket_.reset(new WebSocketHandler());
  server_->addWebSocketHandler("/websocket", *websocket_);

  map_service_.reset(new MapService(FLAGS_dreamview_map));
  sim_world_updater_.reset(
      new SimulationWorldUpdater(websocket_.get(), map_service_.get()));

  return Status::OK();
}

Status Dreamview::Start() {
  // start ROS timer, one-shot = false, auto-start = true
  timer_ = AdapterManager::CreateTimer(ros::Duration(kSimWorldTimeInterval),
                                       &SimulationWorldUpdater::OnPushTimer,
                                       sim_world_updater_.get());
  return Status::OK();
}

void Dreamview::Stop() {
  server_->close();
}

}  // namespace dreamview
}  // namespace apollo
