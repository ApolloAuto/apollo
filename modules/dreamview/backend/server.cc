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
#include "google/protobuf/util/json_util.h"

#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/dreamview/backend/simulation_world_service.h"
#include "modules/dreamview/backend/websocket.h"

DEFINE_string(static_file_dir, "modules/dreamview/frontend/dist",
              "The path to the dreamview distribution directory. The default "
              "value points to built-in version from the Apollo project.");
DEFINE_int32(server_port, 8888, "The port of backend webserver");

namespace apollo {
namespace dreamview {

using apollo::common::time::AsInt64;
using apollo::common::time::Clock;
using apollo::common::time::millis;

using Json = nlohmann::json;

/**
 * @class SimulationWorldUpdater
 * @brief A wrapper around SimulationWorldService and WebsocketServer to keep
 * pushing SimulationWorld to frontend via websocket while handling the response
 * from frontend.
 */
class SimulationWorldUpdater {
 public:
  /**
   * @brief Constructor with the websocket server port.
   * @param port The port on which the websocket server will be launched.
   */
  explicit SimulationWorldUpdater(int websocket_port)
      : sim_world_service_(),
        websocket_(websocket_port, WebsocketServer::NO_LOG) {
    websocket_.Run();
  }

  /**
   * @brief The callback function to get updates from SimulationWorldService,
   * and push them to the frontend clients via websocket when the periodic timer
   * is triggered.
   * @param event Timer event
   */
  void OnPushTimer(const ros::TimerEvent& event) {
    if (!sim_world_service_.ReadyToPush()) {
      AWARN << "Not sending simulation world as the data is not ready!";
      return;
    }
    auto json = sim_world_service_.GetUpdateAsJson();
    websocket_.SendData(json.dump());
  }

 private:
  SimulationWorldService sim_world_service_;
  WebsocketServer websocket_;
};

}  // namespace dreamview
}  // namespace apollo

/// Time interval, in seconds, between pushing SimulationWorld to frontend.
static constexpr double kTimeInterval = 0.1;

int main(int argc, char** argv) {
  using apollo::common::adapter::AdapterManager;
  using apollo::dreamview::SimulationWorldUpdater;

  ::google::InitGoogleLogging("dreamview");
  ::google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "dreamview");

  // Initialize and run the static file server which serves the
  // dreamview htmls and javascripts.
  CivetServer server({"document_root", FLAGS_static_file_dir, "listening_ports",
                      std::to_string(FLAGS_server_port)});

  // Websocket port number is web server port number + 1.
  SimulationWorldUpdater updater(FLAGS_server_port + 1);
  auto timer = AdapterManager::CreateTimer(ros::Duration(kTimeInterval),
                                           &SimulationWorldUpdater::OnPushTimer,
                                           &updater);

  ros::spin();
  return 0;
}
