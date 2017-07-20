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

#include "modules/dreamview/backend/simulation_world_updater.h"

#include <string>

#include "gflags/gflags.h"
#include "google/protobuf/util/json_util.h"

DEFINE_string(dreamview_map, "modules/dreamview/backend/testdata/garage.bin",
              "file path for the map file to be rendered in frontend.");

namespace apollo {
namespace dreamview {

using google::protobuf::util::MessageToJsonString;
using Json = nlohmann::json;

SimulationWorldUpdater::SimulationWorldUpdater(WebSocketHandler *websocket)
    : map_service_(FLAGS_dreamview_map),
      sim_world_service_(&map_service_),
      websocket_(websocket) {
  websocket_->RegisterMessageHandler(
      "RetrieveMapData",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto iter = json.find("elements");
        if (iter != json.end()) {
          MapElementIds map_element_ids(*iter);
          auto retrieved = map_service_.RetrieveMapElements(map_element_ids);

          std::string retrieved_json_string;
          MessageToJsonString(retrieved, &retrieved_json_string);

          Json response;
          response["type"] = "MapData";
          response["data"] = Json::parse(retrieved_json_string);

          websocket_->SendData(response.dump(), conn);
        }
      });
}

void SimulationWorldUpdater::OnPushTimer(const ros::TimerEvent &event) {
  sim_world_service_.Update();
  if (!sim_world_service_.ReadyToPush()) {
    AWARN << "Not sending simulation world as the data is not ready!";
    return;
  }
  auto json = sim_world_service_.GetUpdateAsJson();
  websocket_->SendData(json.dump());
}

}  // namespace dreamview
}  // namespace apollo
