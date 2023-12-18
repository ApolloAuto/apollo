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
    WebSocketHandler *camera_ws,
    WebSocketHandler *plugin_ws, const MapService *map_service,
    PerceptionCameraUpdater *perception_camera_updater,
    PluginManager *plugin_manager, bool routing_from_file)
    : sim_world_service_(map_service, routing_from_file),
      map_service_(map_service),
      websocket_(websocket),
      map_ws_(map_ws),
      camera_ws_(camera_ws),
      plugin_ws_(plugin_ws),
      perception_camera_updater_(perception_camera_updater),
      plugin_manager_(plugin_manager),
      command_id_(0) {
  RegisterMessageHandlers();
}

void SimulationWorldUpdater::RegisterMessageHandlers() {
  // Send current sim_control status to the new client.
  websocket_->RegisterConnectionReadyHandler(
      [this](WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "SimControlStatus";
        response["enabled"] = SimControlManager::Instance()->IsEnabled();
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
      "SetStartPoint",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto point = json["point"];
        if (!SimControlManager::Instance()->IsEnabled()) {
          AWARN << "Set start point need start sim_control.";
        } else {
          if (!ContainsKey(point, "heading")) {
            AWARN << "Set start point need set heading.";
            return;
          }
          SimControlManager::Instance()->ReSetPoinstion(point["x"], point["y"],
                                               point["heading"]);

          // Send a ActionCommand to clear the trajectory of planning.
          if (isProcessRunning("planning.dag")) {
            auto action_command = std::make_shared<ActionCommand>();
            action_command->set_command_id(++command_id_);
            action_command->set_command(ActionCommandType::CLEAR_PLANNING);
            sim_world_service_.PublishActionCommand(action_command);
          }
        }
      });

  websocket_->RegisterMessageHandler(
      "SendRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto lane_follow_command = std::make_shared<LaneFollowCommand>();
        bool succeed =
            ConstructLaneFollowCommand(json, lane_follow_command.get());
        if (succeed) {
          sim_world_service_.PublishLaneFollowCommand(lane_follow_command);
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::INFO,
                                                   "Lane follow command sent.");
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              "Failed to send a Lane follow command.");
        }
      });

  websocket_->RegisterMessageHandler(
      "SendDefaultCycleRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto task = std::make_shared<Task>();
        auto *cycle_routing_task = task->mutable_cycle_routing_task();
        auto *lane_follow_command =
            cycle_routing_task->mutable_lane_follow_command();
        if (!ContainsKey(json, "cycleNumber") ||
            !json.find("cycleNumber")->is_number()) {
          AERROR
              << "Failed to prepare a cycle lane follow command: Invalid cycle "
                 "number";
          return;
        }
        bool succeed = ConstructLaneFollowCommand(json, lane_follow_command);
        if (succeed) {
          cycle_routing_task->set_cycle_num(
              static_cast<int>(json["cycleNumber"]));
          task->set_task_name("cycle_routing_task");
          task->set_task_type(apollo::task_manager::TaskType::CYCLE_ROUTING);
          sim_world_service_.PublishTask(task);
          AINFO << "The task is : " << task->DebugString();
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::INFO,
              "Default cycle lane follow command sent.");
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              "Failed to send a default cycle lane follow command.");
        }
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

  websocket_->RegisterMessageHandler(
      "SendParkingRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto valet_parking_command = std::make_shared<ValetParkingCommand>();
        bool succeed =
            ConstructValetParkingCommand(json, valet_parking_command.get());
        if (succeed) {
          sim_world_service_.PublishValetParkingCommand(valet_parking_command);
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::INFO, "Valet parking command sent.");
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              "Failed to send a Valet parking command.");
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
              waypoint_list.push_back(GetPointJsonFromLaneWaypoint(waypoint));
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
        if (LoadUserDefinedRoutings(DefaultRoutingFile(), &default_routings_)) {
          for (const auto &landmark : default_routings_.landmark()) {
            Json drouting;
            drouting["name"] = landmark.name();

            Json point_list;
            for (const auto &point : landmark.waypoint()) {
              point_list.push_back(GetPointJsonFromLaneWaypoint(point));
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
      "Reset", [this](const Json &json, WebSocketHandler::Connection *conn) {
        sim_world_service_.SetToClear();
        SimControlManager::Instance()->Reset();
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
            SimControlManager::Instance()->Start();
          } else {
            SimControlManager::Instance()->Stop();
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
              MonitorMessageItem::INFO, "Successfully add a routing.");
          if (!default_routing_) {
            AERROR << "Failed to add a routing" << std::endl;
          }
          Json response = JsonUtil::ProtoToTypedJson("AddDefaultRoutingPath",
                                                     *default_routing_);
          response["routingType"] = json["routingType"];
          websocket_->SendData(conn, response.dump());
        } else {
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   "Failed to add a routing.");
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

  camera_ws_->RegisterMessageHandler(
      "GetCameraChannel",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        std::vector<std::string> channels;
        perception_camera_updater_->GetChannelMsg(&channels);
        Json response({});
        response["data"]["name"] = "GetCameraChannelListSuccess";
        for (unsigned int i = 0; i < channels.size(); i++) {
          response["data"]["info"]["channel"][i] = channels[i];
        }
        camera_ws_->SendData(conn, response.dump());
      });
  camera_ws_->RegisterMessageHandler(
      "ChangeCameraChannel",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        if (!perception_camera_updater_->IsEnabled()) {
          return;
        }
        auto channel_info = json.find("data");
        Json response({});
        if (channel_info == json.end()) {
          AERROR << "Cannot  retrieve channel info with unknown channel.";
          response["type"] = "ChangeCameraChannelFail";
          camera_ws_->SendData(conn, response.dump());
          return;
        }
        std::string channel =
            channel_info->dump().substr(1, channel_info->dump().length() - 2);
        if (perception_camera_updater_->ChangeChannel(channel)) {
          Json response({});
          response["type"] = "ChangeCameraChannelSuccess";
          camera_ws_->SendData(conn, response.dump());
        } else {
          response["type"] = "ChangeCameraChannelFail";
          camera_ws_->SendData(conn, response.dump());
        }
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
  if (!ContainsKey(point, "heading")) {
    if (!map_service_->CheckRoutingPoint(point["x"], point["y"])) {
      result["pointId"] = point["id"];
      result["error"] = "Selected point cannot be a routing point.";
      AWARN << result["error"];
    }
  } else {
    if (!map_service_->CheckRoutingPointWithHeading(point["x"], point["y"],
                                                    point["heading"])) {
      result["pointId"] = point["id"];
      result["error"] = "Selected point cannot be a routing point.";
      AWARN << result["error"];
    }
  }
  return result;
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
  lane_follow_command->set_command_id(++command_id_);

  auto iter = json.find("waypoint");
  if (iter != json.end() && iter->is_array()) {
    auto waypoint = lane_follow_command->mutable_way_point();
    for (size_t i = 0; i < iter->size(); ++i) {
      auto &point = (*iter)[i];
      auto *pose = waypoint->Add();
      if (ContainsKey(point, "heading")) {
        pose->set_heading(point["heading"]);
      }
      pose->set_x(point["x"]);
      pose->set_y(point["y"]);
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
    const Json &json, ValetParkingCommand *valet_parking_command) {
  valet_parking_command->clear_parking_spot_id();
  valet_parking_command->set_command_id(++command_id_);
  if (ContainsKey(json, "parkingInfo")) {
    auto parkingInfo = json["parkingInfo"];
    valet_parking_command->set_parking_spot_id(parkingInfo["parkingSpaceId"]);
  } else {
    return false;
  }
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

bool SimulationWorldUpdater::isProcessRunning(const std::string &process_name) {
  std::stringstream commandStream;
  commandStream << "pgrep -f " << process_name;
  std::string command = commandStream.str();

  FILE *fp = popen(command.c_str(), "r");
  if (fp) {
    char result[128];
    if (fgets(result, sizeof(result), fp) != nullptr) {
      AINFO << process_name << " is running";
      pclose(fp);
      return true;
    } else {
      AINFO << process_name << " is not running";
    }
    pclose(fp);
  }
  return false;
}

bool SimulationWorldUpdater::LoadUserDefinedRoutings(
    const std::string &file_name, google::protobuf::Message *message) {
  if (GetProtoFromASCIIFile(file_name, message)) {
    return true;
  }

  AWARN << "Failed to load routings from " << file_name;
  return false;
}

bool SimulationWorldUpdater::AddDefaultRouting(const Json &json) {
  if (!ContainsKey(json, "name")) {
    AERROR << "Failed to save a routing: routing name not found.";
    return false;
  }

  if (!ContainsKey(json, "point")) {
    AERROR << "Failed to save a routing: routing points not "
              "found.";
    return false;
  }

  if (!ContainsKey(json, "routingType")) {
    AERROR << "Failed to save a routing: routing type not "
              "found.";
    return false;
  }

  std::string name = json["name"];
  auto iter = json.find("point");
  std::string routingType = json["routingType"];
  bool isDefaultRouting = (routingType == "defaultRouting");
  default_routing_ = isDefaultRouting ? default_routings_.add_landmark()
                                      : park_go_routings_.add_landmark();
  default_routing_->clear_name();
  default_routing_->clear_waypoint();
  default_routing_->set_name(name);
  auto *waypoint = default_routing_->mutable_waypoint();
  if (iter != json.end() && iter->is_array()) {
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

}  // namespace dreamview
}  // namespace apollo
