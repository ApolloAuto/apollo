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
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace dreamview {

using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessageItem;
using apollo::routing::RoutingRequest;
using google::protobuf::util::MessageToJsonString;
using Json = nlohmann::json;

SimulationWorldUpdater::SimulationWorldUpdater(WebSocketHandler *websocket,
                                               const MapService *map_service,
                                               bool routing_from_file)
    : sim_world_service_(map_service, routing_from_file),
      map_service_(map_service),
      websocket_(websocket) {

  // Initialize default end point
  CHECK(apollo::common::util::GetProtoFromASCIIFile(
      apollo::hdmap::EndWayPointFile(),
      &default_end_point_));

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

  websocket_->RegisterMessageHandler(
      "SendRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        RoutingRequest routing_request;
        bool succeed = ConstructRoutingRequest(json, &routing_request);
        if (succeed) {
          AdapterManager::FillRoutingRequestHeader(FLAGS_dreamview_module_name,
                                                   &routing_request);
          AdapterManager::PublishRoutingRequest(routing_request);
        }

        // publish message
        if (succeed) {
          sim_world_service_.PublishMessage(
              MonitorMessageItem::LogLevel::MonitorMessageItem_LogLevel_INFO,
              "Routing Request Sent");
        } else {
          sim_world_service_.PublishMessage(
              MonitorMessageItem::LogLevel::MonitorMessageItem_LogLevel_ERROR,
              "Failed to send routing request");
        }
      });
}

bool SimulationWorldUpdater::ConstructRoutingRequest(
      const Json &json,
      RoutingRequest* routing_request) {
  // set start point
  auto start = json["start"];
  if (start.find("x") == start.end() || start.find("y") == start.end()) {
    AERROR << "Failed to prepare a routing request: start point not found";
    return false;
  }
  map_service_->ConstructLaneWayPoint(start["x"], start["y"],
                                      routing_request->mutable_start());

  // set way point(s) if any
  auto iter = json.find("waypoint");
  if (iter != json.end()) {
    auto* waypoint = routing_request->mutable_waypoint();
    for (size_t i = 0; i < iter->size(); ++i) {
      auto& point = (*iter)[i];
      if (!map_service_->ConstructLaneWayPoint(point["x"], point["y"],
                                               waypoint->Add())) {
        waypoint->RemoveLast();
      }
    }
  }

  // set end point
  RoutingRequest::LaneWaypoint *endLane = routing_request->mutable_end();
  if (json["sendDefaultRoute"]) {
    endLane->set_id(default_end_point_.id());
    endLane->set_s(default_end_point_.s());
    auto *pose = endLane->mutable_pose();
    pose->set_x(default_end_point_.pose().x());
    pose->set_y(default_end_point_.pose().y());
  } else {
    auto end = json["end"];
    if (end.find("x") == end.end() || end.find("y") == end.end()) {
      AERROR << "Failed to prepare a routing request: end point not found";
      return false;
    }
    map_service_->ConstructLaneWayPoint(end["x"], end["y"], endLane);
  }

  return true;
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
