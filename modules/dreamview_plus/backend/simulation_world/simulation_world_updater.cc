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

#include "modules/dreamview_plus/backend/simulation_world/simulation_world_updater.h"

#include "google/protobuf/util/json_util.h"

#include "cyber/common/file.h"
#include "modules/common/util/json_util.h"
#include "modules/common/util/map_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/common/fuel_monitor/fuel_monitor_manager.h"
#include "modules/dreamview/backend/common/sim_control_manager/sim_control_manager.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace dreamview {

using apollo::common::monitor::MonitorMessageItem;
using apollo::common::util::ContainsKey;
using apollo::common::util::JsonUtil;
using apollo::cyber::common::GetProtoFromASCIIFile;
using apollo::cyber::common::SetProtoToASCIIFile;
using apollo::cyber::common::SetStringToASCIIFile;
using apollo::external_command::LaneFollowCommand;
using apollo::external_command::ValetParkingCommand;
using apollo::external_command::ActionCommandType;
using apollo::external_command::ActionCommand;
using apollo::hdmap::DefaultRoutingFile;
using apollo::hdmap::EndWayPointFile;
using apollo::hdmap::ParkGoRoutingFile;
using apollo::localization::LocalizationEstimate;
using apollo::relative_map::NavigationInfo;
using apollo::routing::LaneWaypoint;
using apollo::task_manager::CycleRoutingTask;
using apollo::task_manager::ParkGoRoutingTask;
using apollo::task_manager::ParkingRoutingTask;
using apollo::task_manager::Task;

using Json = nlohmann::json;
using google::protobuf::util::JsonStringToMessage;
using google::protobuf::util::MessageToJsonString;

SimulationWorldUpdater::SimulationWorldUpdater(
    WebSocketHandler *websocket, WebSocketHandler *map_ws,
    WebSocketHandler *plugin_ws, const MapService *map_service,
    PluginManager *plugin_manager, WebSocketHandler *sim_world_ws,
    HMI *hmi, bool routing_from_file)
    : sim_world_service_(map_service, routing_from_file),
      map_service_(map_service),
      websocket_(websocket),
      map_ws_(map_ws),
      plugin_ws_(plugin_ws),
      // perception_camera_updater_(perception_camera_updater),
      plugin_manager_(plugin_manager),
      sim_world_ws_(sim_world_ws),
      hmi_(hmi),
      command_id_(0) {
  RegisterRoutingMessageHandlers();
  RegisterMessageHandlers();
}

void SimulationWorldUpdater::RegisterRoutingMessageHandlers() {
  websocket_->RegisterMessageHandler(
      "SendRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to send routing request: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        Json end;
        std::vector<std::string> json_path = {"data", "info", "end"};
        if (!JsonUtil::GetJsonByPath(json, json_path, &end)) {
          AERROR << "Failed to find end: end not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss end";
          websocket_->SendData(conn, response.dump());
          return;
        }
        std::string parking_space_id =
            map_service_->GetParkingSpaceId(end["x"], end["y"]);
        if (parking_space_id != "-1") {
          // If the parking space ID can be obtained, it means it is a parking
          // routing
          auto valet_parking_command = std::make_shared<ValetParkingCommand>();
          bool succeed = ConstructValetParkingCommand(
              json, valet_parking_command.get(), parking_space_id);
          if (succeed) {
            sim_world_service_.PublishValetParkingCommand(
                valet_parking_command);
            sim_world_service_.PublishMonitorMessage(
                MonitorMessageItem::INFO, "Valet parking command sent.");
            response["data"]["info"]["code"] = 0;
            response["data"]["info"]["message"] = "Success";
          } else {
            sim_world_service_.PublishMonitorMessage(
                MonitorMessageItem::ERROR,
                "Failed to send a Valet parking command.");
            response["data"]["info"]["code"] = -1;
            response["data"]["info"]["message"] =
                "Failed to send a Valet parking command";
          }
          websocket_->SendData(conn, response.dump());
        } else {
          // Otherwise, it is a normal routing
          auto lane_follow_command = std::make_shared<LaneFollowCommand>();
          bool succeed =
              ConstructLaneFollowCommand(json, lane_follow_command.get());
          if (succeed) {
            sim_world_service_.PublishLaneFollowCommand(lane_follow_command);
            sim_world_service_.PublishMonitorMessage(
                MonitorMessageItem::INFO, "Lane follow command sent.");
            response["data"]["info"]["code"] = 0;
            response["data"]["info"]["message"] = "Success";
          } else {
            sim_world_service_.PublishMonitorMessage(
                MonitorMessageItem::ERROR,
                "Failed to send a Lane follow command.");
            response["data"]["info"]["code"] = -1;
            response["data"]["info"]["message"] =
                "Failed to send a Lane follow command";
          }
          websocket_->SendData(conn, response.dump());
        }
      });

  websocket_->RegisterMessageHandler(
      "SendDefaultCycleRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to prepare a cycle lane follow command: requestId "
                    "not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        Json info;
        std::vector<std::string> json_path = {"data", "info"};
        if (!JsonUtil::GetJsonByPath(json, json_path, &info)) {
          AERROR << "Failed to prepare a cycle lane follow command: info not "
                    "found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss info";
          websocket_->SendData(conn, response.dump());
          return;
        }
        auto task = std::make_shared<Task>();
        auto *cycle_routing_task = task->mutable_cycle_routing_task();
        auto *lane_follow_command =
            cycle_routing_task->mutable_lane_follow_command();
        if (!ContainsKey(info, "cycleNumber") ||
            !info.find("cycleNumber")->is_number()) {
          AERROR
              << "Failed to prepare a cycle lane follow command: Invalid cycle "
                 "number";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Invalid cycle number";
          websocket_->SendData(conn, response.dump());
          return;
        }
        bool succeed = ConstructLaneFollowCommand(json, lane_follow_command);
        if (succeed) {
          cycle_routing_task->set_cycle_num(
              static_cast<int>(info["cycleNumber"]));
          task->set_task_name("cycle_routing_task");
          task->set_task_type(apollo::task_manager::TaskType::CYCLE_ROUTING);
          sim_world_service_.PublishTask(task);
          AINFO << "The task is : " << task->DebugString();
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::INFO,
              "Default cycle lane follow command sent.");
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = "Success";
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              "Failed to send a default cycle lane follow command.");
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "Failed to send a default cycle lane follow command";
        }
        websocket_->SendData(conn, response.dump());
      });

  // websocket_->RegisterMessageHandler(
  //     "SendParkGoRoutingRequest",
  //     [this](const Json &json, WebSocketHandler::Connection *conn) {
  //       auto task = std::make_shared<Task>();
  //       auto *park_go_routing_task = task->mutable_park_go_routing_task();
  //       if (!ContainsKey(json, "parkTime") ||
  //           !json.find("parkTime")->is_number()) {
  //         AERROR << "Failed to prepare a park go routing request: Invalid
  //         park "
  //                   "time";
  //         return;
  //       }
  //       bool succeed = ConstructRoutingRequest(
  //           json, park_go_routing_task->mutable_routing_request());
  //       if (succeed) {
  //         park_go_routing_task->set_park_time(
  //             static_cast<int>(json["parkTime"]));
  //         task->set_task_name("park_go_routing_task");
  //         task->set_task_type(apollo::task_manager::TaskType::PARK_GO_ROUTING);
  //         sim_world_service_.PublishTask(task);
  //         AINFO << "The task is : " << task->DebugString();
  //         sim_world_service_.PublishMonitorMessage(
  //             MonitorMessageItem::INFO, "Park go routing request sent.");
  //       } else {
  //         sim_world_service_.PublishMonitorMessage(
  //             MonitorMessageItem::ERROR,
  //             "Failed to send a park go routing request.");
  //       }
  //     });

  // websocket_->RegisterMessageHandler(
  //     "SendParkingRoutingRequest",
  //     [this](const Json &json, WebSocketHandler::Connection *conn) {
  //       Json response;
  //       response["action"] = "response";
  //       std::string request_id;
  //       if (!JsonUtil::GetStringByPath(json,
  //           "data.requestId", &request_id)) {
  //         AERROR
  //             << "Failed to send parking routing request:
  //           requestId not found.";
  //         response["data"]["info"]["code"] = -1;
  //         response["data"]["info"]["message"] = "Miss requestId";
  //         websocket_->SendData(conn, response.dump());
  //         return;
  //       }
  //       response["data"]["requestId"] = request_id;
  //       auto valet_parking_command = std::make_shared<ValetParkingCommand>();
  //       bool succeed =
  //           ConstructValetParkingCommand(json, valet_parking_command.get());
  //       if (succeed) {
  //         sim_world_service_.PublishValetParkingCommand(valet_parking_command);
  //         sim_world_service_.PublishMonitorMessage(
  //             MonitorMessageItem::INFO, "Valet parking command sent.");
  //         response["data"]["info"]["code"] = 0;
  //         response["data"]["info"]["message"] = "Success";
  //       } else {
  //         sim_world_service_.PublishMonitorMessage(
  //             MonitorMessageItem::ERROR,
  //             "Failed to send a Valet parking command.");
  //         response["data"]["info"]["code"] = -1;
  //         response["data"]["info"]["message"] =
  //             "Failed to send a Valet parking command";
  //       }
  //       websocket_->SendData(conn, response.dump());
  //     });

  websocket_->RegisterMessageHandler(
      "GetDefaultRoutings",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to get default routings: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        response["data"]["info"]["threshold"] =
            FLAGS_loop_routing_end_to_start_distance_threshold;
        Json default_routing_list = Json::array();
        if (LoadUserDefinedRoutings(DefaultRoutingFile(), &default_routings_)) {
          for (const auto &landmark : default_routings_.landmark()) {
            Json drouting;
            drouting["name"] = landmark.name();

            Json point_list;
            for (const auto &point : landmark.waypoint()) {
              point_list.push_back(GetPointJsonFromLaneWaypoint(point));
            }
            drouting["point"] = point_list;
            if (landmark.has_cycle_number()) {
              drouting["cycleNumber"] = landmark.cycle_number();
            }

            default_routing_list.push_back(drouting);
          }
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = "Success";
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              "Failed to load default "
              "routing. Please make sure the "
              "file exists at " +
                  DefaultRoutingFile());
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "Failed to load default "
              "routing. Please make sure the "
              "file exists at " +
              DefaultRoutingFile();
        }
        response["data"]["info"]["data"]["defaultRoutings"] =
            default_routing_list;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "GetParkAndGoRoutings",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "ParkAndGoRoutings";
        Json park_go_routing_list = Json::array();
        if (LoadUserDefinedRoutings(ParkGoRoutingFile(), &park_go_routings_)) {
          for (const auto &landmark : park_go_routings_.landmark()) {
            Json park_go_routing;
            park_go_routing["name"] = landmark.name();

            Json point_list;
            for (const auto &point : landmark.waypoint()) {
              point_list.push_back(GetPointJsonFromLaneWaypoint(point));
            }
            park_go_routing["point"] = point_list;
            park_go_routing_list.push_back(park_go_routing);
          }
          //  } else {
          //   sim_world_service_.PublishMonitorMessage(
          //       MonitorMessageItem::ERROR,
          //       "Failed to load park go "
          //       "routing. Please make sure the "
          //       "file exists at " +
          //           ParkGoRoutingFile());
        }
        response["parkAndGoRoutings"] = park_go_routing_list;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "SaveDefaultRouting",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to save default routing: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;

        bool succeed = AddDefaultRouting(json);
        if (succeed) {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::INFO, "Successfully add a routing.");
          if (!default_routing_) {
            AERROR << "Failed to add a routing" << std::endl;
          }
          Json ret_data = JsonUtil::ProtoToTypedJson("AddDefaultRoutingPath",
                                                     *default_routing_);
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = "Success";
          response["data"]["info"]["data"] = ret_data["data"];
        } else {
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   "Failed to add a routing.");
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Failed to add a routing";
        }
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "DeleteDefaultRouting",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["action"] = "response";
        std::string request_id, routing_name;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to delete default routing: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        if (!JsonUtil::GetStringByPath(json, "data.info.name", &routing_name)) {
          AERROR << "Failed to delete default routing: name not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss name";
          websocket_->SendData(conn, response.dump());
          return;
        }
        bool ret = DeleteDefaultRouting(routing_name);
        if (!ret) {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "Failed to delete default routing";
        } else {
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = "Success";
        }
        websocket_->SendData(conn, response.dump());
      });
}

void SimulationWorldUpdater::RegisterMessageHandlers() {
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
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to check routing point: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        Json result;
        if (CheckRoutingPoint(json, result)) {
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = result["message"];
          response["data"]["info"]["data"]["isLegal"] = result["isLegal"];
        } else {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = result["message"];
        }
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "SetStartPoint",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        std::string request_id;
        response["action"] = "response";
        response["data"]["info"] = {};
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Miss requestId to reset message";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        auto sim_control_manager = SimControlManager::Instance();
        if (!sim_control_manager->IsEnabled()) {
          AWARN << "Set start point need start sim_control.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "Failed to set star point for sim control is not enabled!";
          websocket_->SendData(conn, response.dump());
          return;
        }
        Json point;
        double x, y, heading;
        std::vector<std::string> json_path = {"data", "info", "point"};
        if (!JsonUtil::GetJsonByPath(json, json_path, &point)) {
          apollo::common::PointENU point_enu;
          if (!map_service_->GetStartPoint(&point_enu)) {
            response["data"]["info"]["code"] = -1;
            response["data"]["info"]["message"] =
                "Failed to get a dummy start point from map!";
            websocket_->SendData(conn, response.dump());
            return;
          }
          x = point_enu.x();
          y = point_enu.y();
          heading = 0.0;
          double s = 0.0;
          map_service_->GetPoseWithRegardToLane(x, y, &heading, &s);
        } else {
          if (!JsonUtil::GetNumber(point, "heading", &heading) ||
              !JsonUtil::GetNumber(point, "x", &x) ||
              !JsonUtil::GetNumber(point, "y", &y)) {
            AWARN << "Set start point need start sim_control.";
            response["data"]["info"]["code"] = -1;
            response["data"]["info"]["message"] =
                "Failed to set star point for point field is invalid";
            websocket_->SendData(conn, response.dump());
            return;
          }
        }
        sim_control_manager->ReSetPoinstion(x, y, heading);
        // Send a ActionCommand to clear the trajectory of planning.
        if (hmi_->isProcessRunning("planning.dag")) {
          auto action_command = std::make_shared<ActionCommand>();
          action_command->set_command_id(++command_id_);
          action_command->set_command(ActionCommandType::CLEAR_PLANNING);
          sim_world_service_.PublishActionCommand(action_command);
        }
        std::string start_str = std::to_string(x) + "," + std::to_string(y) +
                                "," + std::to_string(heading);
        std::string start_point_file =
            FLAGS_map_dir + "/" + FLAGS_current_start_point_filename;
        if (!SetStringToASCIIFile(start_str, start_point_file)) {
          AERROR << "Failed to set start point to ascii file "
                 << start_point_file;
        }
        response["data"]["info"]["code"] = 0;
        websocket_->SendData(conn, response.dump());
        return;
      });

  websocket_->RegisterMessageHandler(
      "GetMapElementIds",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        std::string request_id;
        response["action"] = "response";
        response["data"]["info"] = {};
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Miss requestId to reset message";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        double radius;
        if (!JsonUtil::GetNumberByPath(json, "data.info.radius", &radius)) {
          AWARN << "Collect map element ids need radius info.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "Failed to collect map element ids for radius field is missed.";
          websocket_->SendData(conn, response.dump());
          return;
        }
        Json point;
        apollo::common::PointENU point_enu;
        std::vector<std::string> json_path = {"data", "info", "point"};
        if (!JsonUtil::GetJsonByPath(json, json_path, &point)) {
          AERROR << "Miss point field to set start point,use map service to "
                    "find start point.";
          if (!map_service_->GetStartPoint(&point_enu)) {
            response["data"]["info"]["code"] = -1;
            response["data"]["info"]["message"] =
                "Failed to collect map element ids for failed to get a dummy "
                "start point from map!";
            websocket_->SendData(conn, response.dump());
            return;
          }
        } else {
          double x, y, z;
          if (!JsonUtil::GetNumber(point, "x", &x) ||
              !JsonUtil::GetNumber(point, "y", &y) ||
              !JsonUtil::GetNumberByPath(json, "data.info.radius", &radius)) {
            response["data"]["info"]["code"] = -1;
            response["data"]["info"]["message"] =
                "Failed to collect map element ids for point field is invalid.";
            websocket_->SendData(conn, response.dump());
            return;
          }
          point_enu.set_x(x);
          point_enu.set_y(y);
          if (JsonUtil::GetNumber(point, "z", &z)) {
            point_enu.set_z(z);
          }
        }
        MapElementIds map_element_ids;
        map_service_->CollectMapElementIds(point_enu, radius, &map_element_ids);
        response["data"]["info"]["data"] =
            JsonUtil::ProtoToJson(map_element_ids);
        response["data"]["info"]["code"] = 0;
        websocket_->SendData(conn, response.dump());
        return;
      });

  websocket_->RegisterMessageHandler(
      "GetMapElementsByIds",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        std::string request_id;
        response["action"] = "response";
        response["data"]["info"] = {};
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Miss requestId to reset message";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        Json element_ids_json;
        if (!JsonUtil::GetJsonByPath(json,
                                     {"data", "info", "param", "mapElementIds"},
                                     &element_ids_json)) {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss map elementIds";
          websocket_->SendData(conn, response.dump());
          return;
        }
        MapElementIds map_element_ids;
        if (!JsonStringToMessage(element_ids_json.dump(), &map_element_ids)
                 .ok()) {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "Failed to parse elementIds from json.";
          websocket_->SendData(conn, response.dump());
          return;
        }
        auto retrieved = map_service_->RetrieveMapElements(map_element_ids);
        // std::string retrieved_map_string;
        // retrieved.SerializeToString(&retrieved_map_string);
        response["data"]["info"]["data"] = JsonUtil::ProtoToJson(retrieved);
        response["data"]["info"]["code"] = 0;
        websocket_->SendData(conn, response.dump());
        return;
      });

  websocket_->RegisterMessageHandler(
      "GetStartPoint",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        std::string request_id;
        response["action"] = "response";
        response["data"]["info"] = {};
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Miss requestId to reset message";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        Object adc = sim_world_service_.world().auto_driving_car();
        if (!adc.has_position_x() || !adc.has_position_y()) {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss adc position x,y val";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["info"]["code"] = 0;
        Json start_point({});
        start_point["x"] = adc.position_x();
        start_point["y"] = adc.position_y();
        if (adc.has_heading()) {
          start_point["heading"] = adc.heading();
        }
        response["data"]["info"]["data"] = start_point;
        websocket_->SendData(conn, response.dump());
        return;
      });

  websocket_->RegisterMessageHandler(
      "RequestRoutePath",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to request route path: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        Json result = sim_world_service_.GetRoutePathAsJson();
        response["data"]["info"]["code"] = 0;
        response["data"]["info"]["message"] = "Success";
        response["data"]["info"]["data"] = result;
        websocket_->SendData(conn, response.dump());
      });

  // websocket_->RegisterMessageHandler(
  //     "GetDefaultEndPoint",
  //     [this](const Json &json, WebSocketHandler::Connection *conn) {
  //       Json response;
  //       response["action"] = "response";
  //       std::string request_id;
  //       if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id))
  //       {
  //         AERROR << "Failed to get default end point: requestId not found.";
  //         response["data"]["info"]["code"] = -1;
  //         response["data"]["info"]["message"] = "Miss requestId";
  //         websocket_->SendData(conn, response.dump());
  //         return;
  //       }
  //       response["data"]["requestId"] = request_id;

  //       Json poi_list = Json::array();
  //       if (LoadPOI()) {
  //         for (const auto &landmark : poi_.landmark()) {
  //           Json place;
  //           place["name"] = landmark.name();

  //           Json parking_info =
  //               apollo::common::util::JsonUtil::ProtoToTypedJson(
  //                   "parkingInfo", landmark.parking_info());
  //           place["parkingInfo"] = parking_info["data"];

  //           Json waypoint_list;
  //           for (const auto &waypoint : landmark.waypoint()) {
  //             waypoint_list.push_back(GetPointJsonFromLaneWaypoint(waypoint));
  //           }
  //           place["waypoint"] = waypoint_list;

  //           poi_list.push_back(place);
  //         }
  //         response["data"]["info"]["code"] = 0;
  //         response["data"]["info"]["message"] = "Success";
  //       } else {
  //         sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
  //                                                  "Failed to load default "
  //                                                  "POI. Please make sure the
  //                                                  " "file exists at " +
  //                                                      EndWayPointFile());
  //         response["data"]["info"]["code"] = -1;
  //         response["data"]["info"]["message"] =  "Failed to load default "
  //                                                 "POI. Please make sure the
  //                                                 " "file exists at " +
  //                                                 EndWayPointFile();
  //       }
  //       response["data"]["info"]["poi"] = poi_list;
  //       websocket_->SendData(conn, response.dump());
  //     });

  websocket_->RegisterMessageHandler(
      "Reset", [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response({});
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Miss requestId to reset message";
          response["action"] = "response";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        sim_world_service_.SetToClear();
        SimControlManager::Instance()->Reset();
        response["action"] = "response";
        response["data"]["info"]["code"] = 0;
        response["data"]["info"]["message"] = "Success";
        response["data"]["requestId"] = request_id;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "Dump", [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response({});
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Miss requestId to dump message";
          response["action"] = "response";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        sim_world_service_.DumpMessages();
        response["action"] = "response";
        response["data"]["info"]["code"] = 0;
        response["data"]["info"]["message"] = "Success";
        response["data"]["requestId"] = request_id;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "GetParkingRoutingDistance",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "ParkingRoutingDistance";
        response["threshold"] = FLAGS_parking_routing_distance_threshold;
        websocket_->SendData(conn, response.dump());
      });

  // websocket_->RegisterMessageHandler(
  //     "ModifyDefaultRoutingCycleNumber",
  //     [this](const Json &json, WebSocketHandler::Connection *conn) {
  //       Json response;
  //       response["action"] = "response";
  //       std::string request_id;
  //       if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id))
  //       {
  //         AERROR << "Failed to save default routing: requestId not found.";
  //         response["data"]["info"]["code"] = -1;
  //         response["data"]["info"]["message"] = "Miss requestId";
  //         websocket_->SendData(conn, response.dump());
  //         return;
  //       }
  //       response["data"]["requestId"] = request_id;
  //       std::string routing_name;
  //       if (!JsonUtil::GetStringByPath(json, "data.info.name",
  //       &routing_name)) {
  //         AERROR << "Failed to delete default routing: name not found.";
  //         response["data"]["info"]["code"] = -1;
  //         response["data"]["info"]["message"] = "Miss name";
  //         websocket_->SendData(conn, response.dump());
  //         return;
  //       } else if (!ContainsKey(json["data"]["info"], "cycleNumber") ||
  //                  !json["data"]["info"].find("cycleNumber")->is_number()) {
  //         AERROR
  //             << "Failed to prepare a cycle lane follow command: Invalid
  //             cycle "
  //                "number";
  //         response["data"]["info"]["code"] = -1;
  //         response["data"]["info"]["message"] = "Invalid cycle number";
  //         websocket_->SendData(conn, response.dump());
  //         return;
  //       }
  //       int cycle_number =
  //           static_cast<int>(json["data"]["info"]["cycleNumber"]);
  //       bool ret = ModifyCycleNumber(routing_name, cycle_number);
  //       if (!ret) {
  //         response["data"]["info"]["code"] = -1;
  //         response["data"]["info"]["message"] =
  //             "Failed to modify default routing cycle number";
  //       } else {
  //         response["data"]["info"]["code"] = 0;
  //         response["data"]["info"]["message"] = "Success";
  //       }
  //       websocket_->SendData(conn, response.dump());
  //     });

  websocket_->RegisterMessageHandler(
      "CheckCycleRouting",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response, result;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to check cycle routing: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        if (CheckCycleRouting(json, result)) {
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["data"]["isCycle"] = result["isCycle"];
          response["data"]["info"]["message"] = result["message"];
        } else {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = result["message"];
        }
        websocket_->SendData(conn, response.dump());
      });

  plugin_ws_->RegisterMessageHandler(
      "PluginRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        if (!plugin_manager_->IsEnabled()) {
          return;
        }
        auto iter = json.find("data");
        if (iter == json.end()) {
          AERROR << "Failed to get plugin msg!";
          return;
        }
        if (!plugin_manager_->SendMsgToPlugin(iter->dump())) {
          AERROR << "Failed to send msg to plugin";
        }
      });
}

bool SimulationWorldUpdater::CheckRoutingPoint(const nlohmann::json &json,
                                               nlohmann::json &result) {
  Json point;
  std::vector<std::string> json_path = {"data", "info", "point"};
  if (!JsonUtil::GetJsonByPath(json, json_path, &point)) {
    result["message"] = "Failed to check routing point: point not found.";
    AERROR << result["message"];
    return false;
  }

  if (!ValidateCoordinate(point) || !ContainsKey(point, "id")) {
    result["message"] = "Failed to check routing point: invalid point.";
    AERROR << result["message"];
    return false;
  }
  if (!ContainsKey(point, "heading")) {
    if (!map_service_->CheckRoutingPoint(point["x"], point["y"])) {
      result["pointId"] = point["id"];
      result["isLegal"] = 0;
      result["message"] = "Selected point cannot be a routing point.";
      AWARN << result["message"];
      return true;
    }
  } else {
    if (!map_service_->CheckRoutingPointWithHeading(point["x"], point["y"],
                                                    point["heading"])) {
      result["pointId"] = point["id"];
      result["isLegal"] = 0;
      result["message"] = "Selected point cannot be a routing point.";
      AWARN << result["message"];
      return true;
    }
  }
  result["isLegal"] = 1;
  result["message"] = "Success";
  return true;
}

Json SimulationWorldUpdater::GetPointJsonFromLaneWaypoint(
    const apollo::routing::LaneWaypoint &waypoint) {
  Json point;
  point["x"] = waypoint.pose().x();
  point["y"] = waypoint.pose().y();
  if (waypoint.has_heading()) {
    point["heading"] = waypoint.heading();
  }
  return point;
}

bool SimulationWorldUpdater::ConstructLaneWayPoint(const Json &point,
                                                   LaneWaypoint *laneWayPoint,
                                                   std::string description) {
  if (ContainsKey(point, "heading")) {
    if (!map_service_->ConstructLaneWayPointWithHeading(
            point["x"], point["y"], point["heading"], laneWayPoint)) {
      AERROR << "Failed to prepare a routing request with heading: "
             << point["heading"] << " cannot locate " << description
             << " on map.";
      return false;
    }
  } else {
    if (!map_service_->ConstructLaneWayPoint(point["x"], point["y"],
                                             laneWayPoint)) {
      AERROR << "Failed to prepare a routing request:"
             << " cannot locate " << description << " on map.";
      return false;
    }
  }
  return true;
}

bool SimulationWorldUpdater::ConstructLaneFollowCommand(
    const Json &json, LaneFollowCommand *lane_follow_command) {
  if (!ContainsKey(json, "data")) {
    AERROR << "Failed to prepare a cycle lane follow command: data not found.";
    return false;
  } else if (!ContainsKey(json["data"], "info")) {
    AERROR << "Failed to prepare a cycle lane follow command: info not found.";
    return false;
  }
  lane_follow_command->set_command_id(++command_id_);

  auto iter = json["data"]["info"].find("waypoint");
  if (iter != json["data"]["info"].end() && iter->is_array()) {
    auto waypoint = lane_follow_command->mutable_way_point();
    for (size_t i = 0; i < iter->size(); ++i) {
      auto &point = (*iter)[i];
      auto *pose = waypoint->Add();
      pose->set_x(point["x"]);
      pose->set_y(point["y"]);
    }
  }

  // set end point
  if (!ContainsKey(json["data"]["info"], "end")) {
    AERROR << "Failed to prepare a routing request: end point not found.";
    return false;
  }

  auto end = json["data"]["info"]["end"];
  if (!ValidateCoordinate(end)) {
    AERROR << "Failed to prepare a routing request: invalid end point.";
    return false;
  }
  auto end_pose = lane_follow_command->mutable_end_pose();
  end_pose->Clear();
  if (ContainsKey(end, "heading")) {
    end_pose->set_heading(end["heading"]);
  }
  end_pose->set_x(end["x"]);
  end_pose->set_y(end["y"]);

  AINFO << "Constructed LaneFollowCommand to be sent:\n"
        << lane_follow_command->DebugString();

  return true;
}

bool SimulationWorldUpdater::ConstructValetParkingCommand(
    const Json &json, ValetParkingCommand *valet_parking_command,
    const std::string &parking_space_id) {
  valet_parking_command->clear_parking_spot_id();
  valet_parking_command->set_command_id(++command_id_);
  valet_parking_command->set_parking_spot_id(parking_space_id);
  AINFO << "Constructed ValetParkingCommand to be sent:\n"
        << valet_parking_command->DebugString();
  return true;
}

Json SimulationWorldUpdater::GetConstructRoutingRequestJson(
    const nlohmann::json &start, const nlohmann::json &end) {
  Json result;
  result["start"] = start;
  result["end"] = end;
  return result;
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

void SimulationWorldUpdater::StartStream(const double &time_interval_ms,
                                         const std::string &channel_name,
                                         nlohmann::json *subscribe_param) {
  AINFO << "Start SimulationWorld updater with data sending frequency: "
        << time_interval_ms;
  time_interval_ms_ = time_interval_ms;
  if (time_interval_ms_ > 0) {
    timer_.reset(new cyber::Timer(
        time_interval_ms, [this]() { this->OnTimer(); }, false));
    timer_->Start();
  } else {
    this->OnTimer();
  }
}

void SimulationWorldUpdater::StopStream(const std::string &channel_name) {
  AINFO << "SimulationWorld updater has been stopped";
  if (timer_) {
    timer_->Stop();
  }
}

void SimulationWorldUpdater::OnTimer(const std::string &channel_name) {
  sim_world_service_.Update();
  {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    last_pushed_adc_timestamp_sec_ =
        sim_world_service_.world().auto_driving_car().timestamp_sec();
    sim_world_service_.GetWireFormatString(
        FLAGS_sim_map_radius, &simulation_world_with_planning_data_);
    sim_world_service_.GetRelativeMap().SerializeToString(
        &relative_map_string_);
  }
  PublishMessage();
}

void SimulationWorldUpdater::PublishMessage(const std::string &channel_name) {
  if (!sim_world_service_.ReadyToPush()) {
    AWARN_EVERY(time_interval_ms_);
    return;
  }
  std::string to_send;
  {
    boost::shared_lock<boost::shared_mutex> writer_lock(mutex_);
    to_send = simulation_world_with_planning_data_;
  }
  StreamData stream_data;
  std::string stream_data_string;
  stream_data.set_action("stream");
  stream_data.set_data_name("simworld");
  std::vector<uint8_t> byte_data(to_send.begin(), to_send.end());
  stream_data.set_data(&(byte_data[0]), byte_data.size());
  stream_data.set_type("simworld");
  stream_data.SerializeToString(&stream_data_string);
  sim_world_ws_->BroadcastBinaryData(stream_data_string);
}

bool SimulationWorldUpdater::LoadPOI() {
  if (GetProtoFromASCIIFile(EndWayPointFile(), &poi_)) {
    return true;
  }

  AWARN << "Failed to load default list of POI from " << EndWayPointFile();
  return false;
}

bool SimulationWorldUpdater::LoadUserDefinedRoutings(
    const std::string &file_name, google::protobuf::Message *message) {
  message->Clear();
  if (GetProtoFromASCIIFile(file_name, message)) {
    return true;
  }

  AWARN << "Failed to load routings from " << file_name;
  return false;
}

bool SimulationWorldUpdater::AddDefaultRouting(const Json &json) {
  if (!ContainsKey(json, "data")) {
    AERROR << "Failed to save a routing: data not found.";
    return false;
  }

  if (!ContainsKey(json["data"], "info")) {
    AERROR << "Failed to save a routing: info not found.";
    return false;
  }

  if (!ContainsKey(json["data"]["info"], "name")) {
    AERROR << "Failed to save a routing: routing name not found.";
    return false;
  }

  if (!ContainsKey(json["data"]["info"], "point")) {
    AERROR << "Failed to save a routing: routing points not "
              "found.";
    return false;
  }

  if (!ContainsKey(json["data"]["info"], "routingType")) {
    AERROR << "Failed to save a routing: routing type not "
              "found.";
    return false;
  }
  std::string name = json["data"]["info"]["name"];
  auto iter = json["data"]["info"].find("point");
  std::string routingType = json["data"]["info"]["routingType"];
  bool isDefaultRouting = (routingType == "defaultRouting");
  default_routing_ = isDefaultRouting ? default_routings_.add_landmark()
                                      : park_go_routings_.add_landmark();
  default_routing_->clear_name();
  default_routing_->clear_waypoint();
  default_routing_->set_name(name);
  auto *waypoint = default_routing_->mutable_waypoint();
  if (iter != json["data"]["info"].end() && iter->is_array()) {
    for (size_t i = 0; i < iter->size(); ++i) {
      auto &point = (*iter)[i];
      if (!ValidateCoordinate(point)) {
        AERROR << "Failed to save a routing: invalid waypoint.";
        return false;
      }
      auto *p = waypoint->Add();
      auto *pose = p->mutable_pose();
      pose->set_x(static_cast<double>(point["x"]));
      pose->set_y(static_cast<double>(point["y"]));
      if (ContainsKey(point, "heading")) {
        p->set_heading(point["heading"]);
      }
    }
  }
  if (ContainsKey(json["data"]["info"], "cycleNumber") &&
      !json["data"]["info"].find("cycleNumber")->is_number()) {
    AERROR << "Failed to save a routing: Invalid cycle number";
    return false;
  } else if (ContainsKey(json["data"]["info"], "cycleNumber")) {
    default_routing_->clear_cycle_number();
    default_routing_->set_cycle_number(
        static_cast<int>(json["data"]["info"]["cycleNumber"]));
  }
  AINFO << "User Defined Routing Points to be saved:\n";
  std::string file_name =
      isDefaultRouting ? DefaultRoutingFile() : ParkGoRoutingFile();
  if (!SetProtoToASCIIFile(
          isDefaultRouting ? default_routings_ : park_go_routings_,
          file_name)) {
    AERROR << "Failed to set proto to ascii file " << file_name;
    return false;
  }
  AINFO << "Success in setting proto to file" << file_name;

  return true;
}

bool SimulationWorldUpdater::DeleteDefaultRouting(
    const std::string &routing_name) {
  /*
    According to the interaction, before deleting or adding,
    the list of current default routes must be obtained first,
    so just modify the current "default_routes" as adding, and then rewrite it.
  */

  apollo::routing::POI ret_default_routings;
  for (auto &landmark : default_routings_.landmark()) {
    if (landmark.name() == routing_name) {
      continue;
    }
    ret_default_routings.add_landmark();
    ret_default_routings
        .mutable_landmark(ret_default_routings.landmark_size() - 1)
        ->CopyFrom(landmark);
  }
  default_routings_.clear_landmark();
  default_routings_.mutable_landmark()->CopyFrom(
      ret_default_routings.landmark());

  if (!SetProtoToASCIIFile(default_routings_, DefaultRoutingFile())) {
    AERROR << "Failed to set proto to ascii file " << DefaultRoutingFile();
    return false;
  }
  return true;
}

// bool SimulationWorldUpdater::ModifyCycleNumber(const std::string
// &routing_name,
//                                                const int &cycle_number) {
//   for (int i = 0; i < default_routings_.landmark_size(); i++) {
//     if (default_routings_.landmark(i).name() == routing_name) {
//       default_routings_.mutable_landmark(i)->set_cycle_number(cycle_number);
//       break;
//     }
//   }
//   if (!SetProtoToASCIIFile(default_routings_, DefaultRoutingFile())) {
//     AERROR << "Failed to set proto to ascii file " << DefaultRoutingFile();
//     return false;
//   }
//   return true;
// }

bool SimulationWorldUpdater::CheckCycleRouting(const Json &json,
                                               nlohmann::json &result) {
  Json start_json, end_json;
  std::vector<std::string> json_path = {"data", "info", "start"};
  if (!JsonUtil::GetJsonByPath(json, json_path, &start_json)) {
    AERROR << "Failed to check cycle routing: Miss start point.";
    result["message"] = "Miss start point";
    return false;
  }
  json_path[json_path.size() - 1] = "end";
  if (!JsonUtil::GetJsonByPath(json, json_path, &end_json)) {
    AERROR << "Failed to check cycle routing: Miss end point.";
    result["message"] = "Miss end point";
    return false;
  }
  apollo::external_command::Pose start, end;
  if (JsonStringToMessage(start_json.dump(), &start).ok() &&
      JsonStringToMessage(end_json.dump(), &end).ok()) {
    double x_dis = start.x() - end.x();
    double y_dis = start.y() - end.y();
    if (x_dis * x_dis + y_dis * y_dis <
        FLAGS_threshold_for_destination_check *
            FLAGS_threshold_for_destination_check) {
      result["message"] = "Success";
      result["isCycle"] = 1;
      return true;
    }
    result["message"] = "Unable to form a cycle routing";
    result["isCycle"] = 0;
    return true;
  }
  AERROR << "Failed to parse MapElementIds from json";
  result["message"] = "Failed to parse MapElementIds from json";
  return false;
}

}  // namespace dreamview
}  // namespace apollo
