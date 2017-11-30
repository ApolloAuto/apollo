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

#include "google/protobuf/util/json_util.h"
#include "modules/common/util/map_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace dreamview {

using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::util::GetProtoFromASCIIFile;
using apollo::common::util::ContainsKey;
using apollo::hdmap::EndWayPointFile;
using apollo::routing::RoutingRequest;
using google::protobuf::util::MessageToJsonString;
using Json = nlohmann::json;

SimulationWorldUpdater::SimulationWorldUpdater(WebSocketHandler *websocket,
                                               SimControl *sim_control,
                                               const MapService *map_service,
                                               bool routing_from_file)
    : sim_world_service_(map_service, routing_from_file),
      map_service_(map_service),
      websocket_(websocket),
      sim_control_(sim_control) {

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

          websocket_->SendData(conn, response.dump());
        }
      });

  websocket_->RegisterMessageHandler(
      "RetrieveMapElementsByRadius",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto radius = json.find("radius");
        if (radius == json.end()) {
          AERROR << "Cannot retrieve map elements with unknown radius.";
          return;
        }

        if (!radius->is_number()) {
          AERROR << "Expect radius with type 'number', but was "
                 << radius->type_name();
          return;
        }

        Json response = sim_world_service_.GetMapElements(*radius);
        response["type"] = "MapElements";
        websocket_->SendData(conn, response.dump());
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

        // Publish monitor message.
        if (succeed) {
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::INFO,
                                                   "Routing request Sent");
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR, "Failed to send routing request");
        }
      });

  websocket_->RegisterMessageHandler(
      "RequestSimulationWorld",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        if (!sim_world_service_.ReadyToPush()) {
          AWARN_EVERY(100)
              << "Not sending simulation world as the data is not ready!";
          return;
        }

        bool requestPlanning = false;
        auto planning = json.find("planning");
        if (planning != json.end() && planning->is_boolean()) {
          requestPlanning = json["planning"];
        }

        std::string to_send;
        {
          // Pay the price to copy the data instead of sending data over the
          // wire while holding the lock.
          boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
          to_send = requestPlanning ? simulation_world_with_planning_json_
                                    : simulation_world_json_;
        }
        websocket_->SendData(conn, to_send, true);
      });

  websocket_->RegisterMessageHandler(
      "GetDefaultEndPoint",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "DefaultEndPoint";

        Json poi_list = Json::array();
        if (LoadPOI()) {
          for (const auto &landmark : poi_.landmark()) {
            Json place;
            place["name"] = landmark.name();
            Json waypoint_list;
            for (const auto &waypoint : landmark.waypoint()) {
              Json point;
              point["x"] = waypoint.pose().x();
              point["y"] = waypoint.pose().y();
              waypoint_list.push_back(point);
            }
            place["waypoint"] = waypoint_list;
            poi_list.push_back(place);
          }
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR, "Failed to load default POI. "
              "Please make sure the file exists at " + EndWayPointFile());
        }
        response["poi"] = poi_list;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "Reset", [this](const Json &json, WebSocketHandler::Connection *conn) {
        sim_world_service_.SetToClear();
        sim_control_->ClearPlanning();
      });

  websocket_->RegisterMessageHandler(
      "Dump", [this](const Json &json, WebSocketHandler::Connection *conn) {
        DumpMessage(AdapterManager::GetChassis(), "Chassis");
        DumpMessage(AdapterManager::GetPrediction(), "Prediction");
        DumpMessage(AdapterManager::GetRoutingResponse(), "RoutingResponse");
        DumpMessage(AdapterManager::GetLocalization(), "Localization");
        DumpMessage(AdapterManager::GetPlanning(), "Planning");
      });
}

bool SimulationWorldUpdater::ConstructRoutingRequest(
    const Json &json, RoutingRequest *routing_request) {
  routing_request->clear_waypoint();
  // set start point
  if (!ContainsKey(json, "start")) {
    AERROR << "Failed to prepare a routing request: start point not found.";
    return false;
  }

  auto start = json["start"];
  if (!ValidateCoordinate(start)) {
    AERROR << "Failed to prepare a routing request: invalid start point.";
    return false;
  }
  if (!map_service_->ConstructLaneWayPoint(start["x"], start["y"],
                                           routing_request->add_waypoint())) {
    AERROR << "Failed to prepare a routing request:"
           << " cannot locate start point on map.";
    return false;
  }

  // set way point(s) if any
  auto iter = json.find("waypoint");
  if (iter != json.end() && iter->is_array()) {
    auto *waypoint = routing_request->mutable_waypoint();
    for (size_t i = 0; i < iter->size(); ++i) {
      auto &point = (*iter)[i];
      if (!ValidateCoordinate(point)) {
        AERROR << "Failed to prepare a routing request: invalid waypoint.";
        return false;
      }

      if (!map_service_->ConstructLaneWayPoint(point["x"], point["y"],
                                               waypoint->Add())) {
        waypoint->RemoveLast();
      }
    }
  }

  // set end point
  if (!ContainsKey(json, "end")) {
    AERROR << "Failed to prepare a routing request: end point not found.";
    return false;
  }

  auto end = json["end"];
  if (!ValidateCoordinate(end)) {
    AERROR << "Failed to prepare a routing request: invalid end point.";
    return false;
  }
  if (!map_service_->ConstructLaneWayPoint(end["x"], end["y"],
                                           routing_request->add_waypoint())) {
    AERROR << "Failed to prepare a routing request:"
           << " cannot locate end point on map.";
    return false;
  }

  AINFO << "Constructed RoutingRequest to be sent:\n"
        << routing_request->DebugString();

  return true;
}

bool SimulationWorldUpdater::ValidateCoordinate(const nlohmann::json &json) {
  if (!ContainsKey(json, "x") || !ContainsKey(json, "y")) {
    AERROR << "Failed to find x or y coordinate.";
    return false;
  }
  if (json.find("x")->is_number() && json.find("y")->is_number()) {
    return true;
  }
  AERROR << "Both x and y coordinate should be a number.";
  return false;
}

void SimulationWorldUpdater::Start() {
  // start ROS timer, one-shot = false, auto-start = true
  timer_ =
      AdapterManager::CreateTimer(ros::Duration(kSimWorldTimeIntervalMs / 1000),
                                  &SimulationWorldUpdater::OnTimer, this);
}

void SimulationWorldUpdater::OnTimer(const ros::TimerEvent &event) {
  sim_world_service_.Update();

  {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    Json simulation_world =
        sim_world_service_.GetUpdateAsJson(FLAGS_sim_map_radius);
    simulation_world_json_ = simulation_world.dump();

    simulation_world["planningData"] = sim_world_service_.GetPlanningData();
    simulation_world_with_planning_json_ = simulation_world.dump();
  }
}

bool SimulationWorldUpdater::LoadPOI() {
  if (GetProtoFromASCIIFile(EndWayPointFile(), &poi_)) {
    return true;
  }

  AWARN << "Failed to load default list of POI from " << EndWayPointFile();
  return false;
}

}  // namespace dreamview
}  // namespace apollo
