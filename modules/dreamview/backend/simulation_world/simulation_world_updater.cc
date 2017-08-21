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

#include "modules/dreamview/backend/simulation_world/simulation_world_updater.h"

#include <string>

#include "google/protobuf/util/json_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

namespace apollo {
namespace dreamview {

using apollo::common::adapter::AdapterManager;
using google::protobuf::util::MessageToJsonString;
using Json = nlohmann::json;

SimulationWorldUpdater::SimulationWorldUpdater(WebSocketHandler *websocket,
                                               MapService *map_service,
                                               bool routing_from_file)
    : sim_world_service_(map_service, routing_from_file),
      map_service_(map_service),
      websocket_(websocket) {
  websocket_->RegisterMessageHandler(
      "RetrieveMapData",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto iter = json.find("elements");
        if (iter != json.end()) {
          MapElementIds map_element_ids(*iter);
          auto retrieved = map_service_->RetrieveMapElements(map_element_ids);

          std::string retrieved_json_string;
          MessageToJsonString(retrieved, &retrieved_json_string);

          Json response;
          response["type"] = "MapData";
          response["data"] = Json::parse(retrieved_json_string);

          websocket_->SendData(response.dump(), conn);
        }
      });
}

void SimulationWorldUpdater::Start() {
  // start ROS timer, one-shot = false, auto-start = true
  timer_ =
      AdapterManager::CreateTimer(ros::Duration(kSimWorldTimeInterval),
                                  &SimulationWorldUpdater::OnPushTimer, this);
}

void SimulationWorldUpdater::OnPushTimer(const ros::TimerEvent &event) {
  sim_world_service_.Update();
  if (!sim_world_service_.ReadyToPush()) {
    AWARN << "Not sending simulation world as the data is not ready!";
    return;
  }
  auto json = sim_world_service_.GetUpdateAsJson();
  websocket_->BroadcastData(json.dump());
}

}  // namespace dreamview
}  // namespace apollo
