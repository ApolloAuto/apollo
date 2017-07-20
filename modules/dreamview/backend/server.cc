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

#include <chrono>
#include <sstream>
#include <thread>

#include "CivetServer.h"
#include "gflags/gflags.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/dreamview/backend/simulation_world_updater.h"
#include "modules/dreamview/backend/websocket.h"

DEFINE_string(static_file_dir, "modules/dreamview/frontend/dist",
              "The path to the dreamview distribution directory. The default "
              "value points to built-in version from the Apollo project.");
DEFINE_int32(server_port, 8888, "The port of backend webserver");

/// Time interval, in seconds, between pushing SimulationWorld to frontend.
static constexpr double kTimeInterval = 0.1;

int main(int argc, char **argv) {
  using apollo::common::adapter::AdapterManager;
  using apollo::dreamview::SimulationWorldUpdater;
  using apollo::dreamview::WebSocketHandler;

  ::google::InitGoogleLogging("dreamview");
  ::google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "dreamview");

  // Initialize and run the web server which serves the
  // dreamview htmls and javascripts and handles websocket requests.
  CivetServer server({"document_root", FLAGS_static_file_dir, "listening_ports",
                      std::to_string(FLAGS_server_port)});
  WebSocketHandler websocket;
  server.addWebSocketHandler("/websocket", websocket);

  SimulationWorldUpdater updater(&websocket);
  auto timer = AdapterManager::CreateTimer(ros::Duration(kTimeInterval),
                                           &SimulationWorldUpdater::OnPushTimer,
                                           &updater);

  ros::spin();
  return 0;
}
