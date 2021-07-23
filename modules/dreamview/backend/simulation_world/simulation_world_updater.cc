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

#include "cyber/common/file.h"
#include "modules/common/util/json_util.h"
#include "modules/common/util/map_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/fuel_monitor/fuel_monitor_manager.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace dreamview {

using apollo::common::monitor::MonitorMessageItem;
using apollo::common::util::ContainsKey;
using apollo::common::util::JsonUtil;
using apollo::cyber::common::GetProtoFromASCIIFile;
using apollo::cyber::common::SetProtoToASCIIFile;
using apollo::hdmap::DefaultRoutingFile;
using apollo::hdmap::EndWayPointFile;
using apollo::relative_map::NavigationInfo;
using apollo::routing::RoutingRequest;
using apollo::task_manager::CycleRoutingTask;
using apollo::task_manager::DeadEndRoutingTask;
using apollo::task_manager::ParkingRoutingTask;
using apollo::task_manager::Task;

using Json = nlohmann::json;
using google::protobuf::util::JsonStringToMessage;
using google::protobuf::util::MessageToJsonString;

SimulationWorldUpdater::SimulationWorldUpdater(
    WebSocketHandler *websocket, WebSocketHandler *map_ws,
    WebSocketHandler *camera_ws, SimControl *sim_control,
    const MapService *map_service,
    PerceptionCameraUpdater *perception_camera_updater, bool routing_from_file)
    : sim_world_service_(map_service, routing_from_file),
      map_service_(map_service),
      websocket_(websocket),
      map_ws_(map_ws),
      camera_ws_(camera_ws),
      sim_control_(sim_control),
      perception_camera_updater_(perception_camera_updater) {
  RegisterMessageHandlers();
}

void SimulationWorldUpdater::RegisterMessageHandlers() {
  // Send current sim_control status to the new client.
  websocket_->RegisterConnectionReadyHandler(
      [this](WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "SimControlStatus";
        response["enabled"] = sim_control_->IsEnabled();
        websocket_->SendData(conn, response.dump());
      });

  map_ws_->RegisterMessageHandler(
      "RetrieveMapData",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto iter = json.find("elements");
        if (iter != json.end()) {
          MapElementIds map_element_ids;
          if (JsonStringToMessage(iter->dump(), &map_element_ids).ok()) {
            auto retrieved = map_service_->RetrieveMapElements(map_element_ids);

            std::string retrieved_map_string;
            retrieved.SerializeToString(&retrieved_map_string);

            map_ws_->SendBinaryData(conn, retrieved_map_string, true);
          } else {
            AERROR << "Failed to parse MapElementIds from json";
          }
        }
      });

  map_ws_->RegisterMessageHandler(
      "RetrieveRelativeMapData",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        std::string to_send;
        {
          boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
          to_send = relative_map_string_;
        }
        map_ws_->SendBinaryData(conn, to_send, true);
      });

  websocket_->RegisterMessageHandler(
      "Binary",
      [this](const std::string &data, WebSocketHandler::Connection *conn) {
        // Navigation info in binary format
        auto navigation_info = std::make_shared<NavigationInfo>();
        if (navigation_info->ParseFromString(data)) {
          sim_world_service_.PublishNavigationInfo(navigation_info);
        } else {
          AERROR << "Failed to parse navigation info from string. String size: "
                 << data.size();
        }
      });

  websocket_->RegisterMessageHandler(
      "RetrieveMapElementIdsByRadius",
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

        Json response;
        response["type"] = "MapElementIds";
        response["mapRadius"] = *radius;

        MapElementIds ids;
        sim_world_service_.GetMapElementIds(*radius, &ids);
        std::string elementIds;
        MessageToJsonString(ids, &elementIds);
        response["mapElementIds"] = Json::parse(elementIds);

        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "CheckRoutingPoint",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response = CheckRoutingPoint(json);
        response["type"] = "RoutingPointCheckResult";
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "SendRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto routing_request = std::make_shared<RoutingRequest>();

        bool succeed = ConstructRoutingRequest(json, routing_request.get());
        if (succeed) {
          sim_world_service_.PublishRoutingRequest(routing_request);
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::INFO,
                                                   "Routing request sent.");
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR, "Failed to send a routing request.");
        }
      });

  websocket_->RegisterMessageHandler(
      "SendDeadEndJunctionRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json result = CheckDeadEndJunctionPoints(json);
        if (result.contains("error")) {
          AINFO << result["error"];
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   result["error"]);
        } else {
          auto task = std::make_shared<Task>();
          auto *dead_junction_routing_task =
              task->mutable_dead_end_routing_task();
          bool succeed = ConstructDeadJunctionRoutingTask(
              result, dead_junction_routing_task);
          if (succeed) {
            task->set_task_name("dead_end_junction_routing_task");
            task->set_task_type(
                apollo::task_manager::TaskType::DEAD_END_ROUTING);
            sim_world_service_.PublishTask(task);
            AINFO << task->DebugString();
            sim_world_service_.PublishMonitorMessage(
                MonitorMessageItem::INFO, "dead junction routing task sent.");
          } else {
            sim_world_service_.PublishMonitorMessage(
                MonitorMessageItem::ERROR,
                "Failed to send a dead junction routing task to task manager "
                "module.");
          }
        }
      });

  websocket_->RegisterMessageHandler(
      "SendDefaultCycleRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto task = std::make_shared<Task>();
        auto *cycle_routing_task = task->mutable_cycle_routing_task();
        auto *routing_request = cycle_routing_task->mutable_routing_request();
        if (!ContainsKey(json, "cycleNumber") ||
            !json.find("cycleNumber")->is_number()) {
          AERROR << "Failed to prepare a cycle routing request: Invalid cycle "
                    "number";
          return;
        }
        bool succeed = ConstructRoutingRequest(json, routing_request);
        if (succeed) {
          cycle_routing_task->set_cycle_num(
              static_cast<int>(json["cycleNumber"]));
          task->set_task_name("cycle_routing_task");
          task->set_task_type(apollo::task_manager::TaskType::CYCLE_ROUTING);
          sim_world_service_.PublishTask(task);
          AINFO << "The task is : " << task->DebugString();
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::INFO, "Default cycle routing request sent.");
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              "Failed to send a default cycle routing request.");
        }
      });

  websocket_->RegisterMessageHandler(
      "SendParkingRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto task = std::make_shared<Task>();
        auto *parking_routing_task = task->mutable_parking_routing_task();
        bool succeed = ConstructParkingRoutingTask(json, parking_routing_task);
        // For test routing
        auto routing_request = std::make_shared<RoutingRequest>();
        bool suc = ConstructRoutingRequest(json, routing_request.get());
        if (suc) {
          sim_world_service_.PublishRoutingRequest(routing_request);
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::INFO,
                                                   "Routing request sent.");
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR, "Failed to send a routing request.");
        }
        if (succeed) {
          task->set_task_name("parking_routing_task");
          task->set_task_type(apollo::task_manager::TaskType::PARKING_ROUTING);
          sim_world_service_.PublishTask(task);
          AINFO << task->DebugString();
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::INFO, "parking routing task sent.");
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              "Failed to send a parking routing task to task manager module.");
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

        bool enable_pnc_monitor = false;
        auto planning = json.find("planning");
        if (planning != json.end() && planning->is_boolean()) {
          enable_pnc_monitor = json["planning"];
        }
        std::string to_send;
        {
          // Pay the price to copy the data instead of sending data over the
          // wire while holding the lock.
          boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
          to_send = enable_pnc_monitor ? simulation_world_with_planning_data_
                                       : simulation_world_;
        }
        if (FLAGS_enable_update_size_check && !enable_pnc_monitor &&
            to_send.size() > FLAGS_max_update_size) {
          AWARN << "update size is too big:" << to_send.size();
          return;
        }
        websocket_->SendBinaryData(conn, to_send, true);
      });

  websocket_->RegisterMessageHandler(
      "RequestRoutePath",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response = sim_world_service_.GetRoutePathAsJson();
        response["type"] = "RoutePath";
        websocket_->SendData(conn, response.dump());
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

            Json parking_info =
                apollo::common::util::JsonUtil::ProtoToTypedJson(
                    "parkingInfo", landmark.parking_info());
            place["parkingInfo"] = parking_info["data"];

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
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   "Failed to load default "
                                                   "POI. Please make sure the "
                                                   "file exists at " +
                                                       EndWayPointFile());
        }
        response["poi"] = poi_list;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "GetDefaultRoutings",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "DefaultRoutings";
        response["threshold"] =
            FLAGS_loop_routing_end_to_start_distance_threshold;

        Json default_routing_list = Json::array();
        if (LoadDefaultRoutings()) {
          for (const auto &defaultrouting :
               default_routings_.defaultrouting()) {
            Json drouting;
            drouting["name"] = defaultrouting.name();
            Json point_list;
            for (const auto &point : defaultrouting.point()) {
              Json point_json;
              point_json["x"] = point.x();
              point_json["y"] = point.y();
              point_list.push_back(point_json);
            }
            drouting["point"] = point_list;
            default_routing_list.push_back(drouting);
          }
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              "Failed to load default "
              "routing. Please make sure the "
              "file exists at " +
                  DefaultRoutingFile());
        }
        response["defaultRoutings"] = default_routing_list;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "Reset", [this](const Json &json, WebSocketHandler::Connection *conn) {
        sim_world_service_.SetToClear();
        sim_control_->Reset();
      });

  websocket_->RegisterMessageHandler(
      "Dump", [this](const Json &json, WebSocketHandler::Connection *conn) {
        sim_world_service_.DumpMessages();
      });

  websocket_->RegisterMessageHandler(
      "ToggleSimControl",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto enable = json.find("enable");
        if (enable != json.end() && enable->is_boolean()) {
          if (*enable) {
            sim_control_->Start();
          } else {
            sim_control_->Stop();
          }
        }
      });

  websocket_->RegisterMessageHandler(
      "RequestDataCollectionProgress",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto *monitors = FuelMonitorManager::Instance()->GetCurrentMonitors();
        if (monitors) {
          const auto iter = monitors->find("DataCollectionMonitor");
          if (iter != monitors->end() && iter->second->IsEnabled()) {
            Json response;
            response["type"] = "DataCollectionProgress";
            response["data"] = iter->second->GetProgressAsJson();
            websocket_->SendData(conn, response.dump());
          }
        }
      });

  websocket_->RegisterMessageHandler(
      "GetParkingRoutingDistance",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "ParkingRoutingDistance";
        response["threshold"] = FLAGS_parking_routing_distance_threshold;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "RequestPreprocessProgress",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto *monitors = FuelMonitorManager::Instance()->GetCurrentMonitors();
        if (monitors) {
          const auto iter = monitors->find("PreprocessMonitor");
          if (iter != monitors->end() && iter->second->IsEnabled()) {
            Json response;
            response["type"] = "PreprocessProgress";
            response["data"] = iter->second->GetProgressAsJson();
            websocket_->SendData(conn, response.dump());
          }
        }
      });

  websocket_->RegisterMessageHandler(
      "SaveDefaultRouting",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        bool succeed = AddDefaultRouting(json);
        if (succeed) {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::INFO, "Successfully add default routing.");
          if (!default_routing_) {
            AERROR << "Failed to add a default routing" << std::endl;
          }
          Json response = JsonUtil::ProtoToTypedJson("AddDefaultRoutingPath",
                                                     *default_routing_);
          websocket_->SendData(conn, response.dump());
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR, "Failed to add a default routing.");
        }
      });

  camera_ws_->RegisterMessageHandler(
      "RequestCameraData",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        if (!perception_camera_updater_->IsEnabled()) {
          return;
        }
        std::string to_send;
        perception_camera_updater_->GetUpdate(&to_send);
        camera_ws_->SendBinaryData(conn, to_send, true);
      });
}

Json SimulationWorldUpdater::CheckRoutingPoint(const Json &json) {
  Json result;
  if (!ContainsKey(json, "point")) {
    result["error"] = "Failed to check routing point: point not found.";
    AERROR << result["error"];
    return result;
  }
  auto point = json["point"];
  if (!ValidateCoordinate(point) || !ContainsKey(point, "id")) {
    result["error"] = "Failed to check routing point: invalid point.";
    AERROR << result["error"];
    return result;
  }
  if (!map_service_->CheckRoutingPoint(point["x"], point["y"])) {
    result["pointId"] = point["id"];
    result["error"] = "Selected point cannot be a routing point.";
    AWARN << result["error"];
  }
  return result;
}

Json SimulationWorldUpdater::CheckDeadEndJunctionPoints(const Json &json) {
  Json result;
  if (!ContainsKey(json, "start1")) {
    result["error"] = "Failed to check start point for dead end junction.";
    AERROR << result["error"];
    return result;
  }
  if (!ContainsKey(json, "end2")) {
    result["error"] = "Failed to check end point for dead end junction.";
    AERROR << result["error"];
    return result;
  }
  auto iter = json.find("inLaneIds");
  if (iter == json.end() || !iter->is_array()) {
    result["error"] = "Failed to check start point for dead end junction.";
    return result;
  }
  std::vector<std::string> laneIds;
  auto point = json["start1"];
  for (size_t i = 0; i < iter->size(); ++i) {
    auto &id = (*iter)[i];
    laneIds.push_back(id);
  }
  if (!map_service_->CheckRoutingPointLaneId(point["x"], point["y"], laneIds)) {
    result["error"] = "Error start point for dead end junction.";
  }
  laneIds.clear();
  point = json["end2"];
  iter = json.find("outLaneIds");
  if (iter == json.end() || !iter->is_array()) {
    result["error"] = "Failed to check end point for dead end junction.";
    return result;
  }
  for (size_t i = 0; i < iter->size(); ++i) {
    auto &id = (*iter)[i];
    laneIds.push_back(id);
  }
  if (!map_service_->CheckRoutingPointLaneId(point["x"], point["y"], laneIds)) {
    result["error"] = "Error end point for dead end junction.";
    return result;
  }

  result["routing1"]["start"] = json["start1"];
  result["routing1"]["end"] = json["end1"];
  result["routing2"]["start"] = json["start2"];
  result["routing2"]["end"] = json["end2"];
  result["routing2"]["waypoint"] = json["routingPoint"];
  return result;
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
  if (ContainsKey(start, "heading")) {
    if (!map_service_->ConstructLaneWayPointWithHeading(
            start["x"], start["y"], start["heading"],
            routing_request->add_waypoint())) {
      AERROR << "Failed to prepare a routing request with heading: "
             << start["heading"] << " cannot locate start point on map.";
      return false;
    }
  } else if (ContainsKey(start, "id")) {
    if (!map_service_->ConstructLaneWayPointWithLaneId(
            start["x"], start["y"], start["id"],
            routing_request->add_waypoint())) {
      AERROR << "Failed to prepare a routing request with lane id: "
             << start["id"] << " cannot locate end point on map.";
      return false;
    }
  } else {
    if (!map_service_->ConstructLaneWayPoint(start["x"], start["y"],
                                             routing_request->add_waypoint())) {
      AERROR << "Failed to prepare a routing request:"
             << " cannot locate start point on map.";
      return false;
    }
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
        AERROR << "Failed to construct a LaneWayPoint, skipping.";
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
  if (ContainsKey(end, "id")) {
    if (!map_service_->ConstructLaneWayPointWithLaneId(
            end["x"], end["y"], end["id"], routing_request->add_waypoint())) {
      AERROR << "Failed to prepare a routing request with lane id: "
             << end["id"] << " cannot locate end point on map.";
      return false;
    }
  } else {
    if (!map_service_->ConstructLaneWayPoint(end["x"], end["y"],
                                             routing_request->add_waypoint())) {
      AERROR << "Failed to prepare a routing request:"
             << " cannot locate end point on map.";
      return false;
    }
  }

  // set parking info
  if (ContainsKey(json, "parkingInfo")) {
    auto *requested_parking_info = routing_request->mutable_parking_info();
    if (!JsonStringToMessage(json["parkingInfo"].dump(), requested_parking_info)
             .ok()) {
      AERROR << "Failed to prepare a routing request: invalid parking info."
             << json["parkingInfo"].dump();
      return false;
    }
    if (ContainsKey(json, "cornerPoints")) {
      auto point_iter = json.find("cornerPoints");
      auto *points =
          requested_parking_info->mutable_corner_point()->mutable_point();
      if (point_iter != json.end() && point_iter->is_array()) {
        for (size_t i = 0; i < point_iter->size(); ++i) {
          auto &point = (*point_iter)[i];
          auto *p = points->Add();
          if (!ValidateCoordinate(point)) {
            AERROR << "Failed to add a corner point: invalid corner point.";
            return false;
          }
          p->set_x(static_cast<double>(point["x"]));
          p->set_y(static_cast<double>(point["y"]));
        }
      }
    }
  }

  AINFO << "Constructed RoutingRequest to be sent:\n"
        << routing_request->DebugString();

  return true;
}

bool SimulationWorldUpdater::ConstructParkingRoutingTask(
    const Json &json, ParkingRoutingTask *parking_routing_task) {
  auto *routing_request = parking_routing_task->mutable_routing_request();
  // set parking Space
  if (!ContainsKey(json, "laneWidth")) {
    AERROR << "Failed to prepare a parking routing task: "
           << "lane width not found.";
    return false;
  }
  bool succeed = ConstructRoutingRequest(json, routing_request);
  if (succeed) {
    parking_routing_task->set_lane_width(
        static_cast<double>(json["laneWidth"]));
    return true;
  }
  return false;
}

bool SimulationWorldUpdater::ConstructDeadJunctionRoutingTask(
    const Json &json, DeadEndRoutingTask *dead_junction_routing_task) {
  auto *routing_request_in =
      dead_junction_routing_task->mutable_routing_request_in();
  bool succeed = ConstructRoutingRequest(json["routing1"], routing_request_in);
  if (!succeed) {
    AERROR << "Failed to construct the first routing request for dead end "
              "junction routing task";
    return false;
  }
  auto *routing_request_out =
      dead_junction_routing_task->mutable_routing_request_out();
  succeed = ConstructRoutingRequest(json["routing2"], routing_request_out);
  if (!succeed) {
    AERROR << "Failed to construct the second routing request for dead end "
              "junction routing task";
    return false;
  }
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
  timer_.reset(new cyber::Timer(
      kSimWorldTimeIntervalMs, [this]() { this->OnTimer(); }, false));
  timer_->Start();
}

void SimulationWorldUpdater::OnTimer() {
  sim_world_service_.Update();

  {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    last_pushed_adc_timestamp_sec_ =
        sim_world_service_.world().auto_driving_car().timestamp_sec();
    sim_world_service_.GetWireFormatString(
        FLAGS_sim_map_radius, &simulation_world_,
        &simulation_world_with_planning_data_);
    sim_world_service_.GetRelativeMap().SerializeToString(
        &relative_map_string_);
  }
}

bool SimulationWorldUpdater::LoadPOI() {
  if (GetProtoFromASCIIFile(EndWayPointFile(), &poi_)) {
    return true;
  }

  AWARN << "Failed to load default list of POI from " << EndWayPointFile();
  return false;
}

bool SimulationWorldUpdater::LoadDefaultRoutings() {
  if (GetProtoFromASCIIFile(DefaultRoutingFile(), &default_routings_)) {
    return true;
  }

  AWARN << "Failed to load default routings of DefaultRoutings from "
        << DefaultRoutingFile();
  return false;
}

bool SimulationWorldUpdater::AddDefaultRouting(const Json &json) {
  if (!ContainsKey(json, "name")) {
    AERROR << "Failed to save a default routing: routing name not found.";
    return false;
  }

  if (!ContainsKey(json, "point")) {
    AERROR << "Failed to save a default routing: default routing points not "
              "found.";
    return false;
  }

  std::string name = json["name"];
  auto iter = json.find("point");
  default_routing_ = default_routings_.add_defaultrouting();
  default_routing_->clear_name();
  default_routing_->clear_point();
  default_routing_->set_name(name);
  auto *waypoint = default_routing_->mutable_point();
  if (iter != json.end() && iter->is_array()) {
    for (size_t i = 0; i < iter->size(); ++i) {
      auto &point = (*iter)[i];
      auto *p = waypoint->Add();
      if (!ValidateCoordinate(point)) {
        AERROR << "Failed to save a default routing: invalid waypoint.";
        return false;
      }
      p->set_x(static_cast<double>(point["x"]));
      p->set_y(static_cast<double>(point["y"]));
    }
  }
  AINFO << "Default Routing Points to be saved:\n";
  std::string file_name = DefaultRoutingFile();
  if (!SetProtoToASCIIFile(default_routings_, file_name)) {
    AERROR << "Failed to set proto to ascii file " << file_name;
    return false;
  }
  AINFO << "Success in setting proto to cycle_routing file" << file_name;

  return true;
}

}  // namespace dreamview
}  // namespace apollo
