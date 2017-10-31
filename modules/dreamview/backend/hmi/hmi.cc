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

#include "modules/dreamview/backend/hmi/hmi.h"

#include <cstdlib>
#include "gflags/gflags.h"
#include "google/protobuf/util/json_util.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/util/util.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

DEFINE_string(global_flagfile, "modules/common/data/global_flagfile.txt",
              "Global flagfile shared by all modules.");

namespace apollo {
namespace dreamview {

using apollo::canbus::Chassis;
using apollo::common::adapter::AdapterManager;
using apollo::control::DrivingAction;
using google::protobuf::Map;
using Json = WebSocketHandler::Json;

namespace {

std::string ProtoToTypedJson(const std::string &json_type,
                             const google::protobuf::Message &proto) {
  std::string json_string;
  google::protobuf::util::MessageToJsonString(proto, &json_string);

  Json json_obj;
  json_obj["type"] = json_type;
  json_obj["data"] = Json::parse(json_string);
  return json_obj.dump();
}

}  // namespace

HMI::HMI(WebSocketHandler *websocket) : websocket_(websocket) {
  CHECK(common::util::GetProtoFromFile(FLAGS_hmi_config_filename, &config_))
      << "Unable to parse HMI config file " << FLAGS_hmi_config_filename;

  // Register websocket message handlers.
  if (websocket_) {
    // Send current config and status to new HMI client.
    websocket_->RegisterConnectionReadyHandler(
        [this](WebSocketHandler::Connection *conn) {
          websocket_->SendData(conn, ProtoToTypedJson("HMIConfig", config_));
          websocket_->SendData(conn, ProtoToTypedJson("HMIStatus", status_));
        });

    // HMI client asks for executing module command.
    websocket_->RegisterMessageHandler(
        "ExecuteModuleCommand",
        [this](const Json &json, WebSocketHandler::Connection *conn) {
          // json should contain
          // {module: "module_name", command: "command_name"}.
          const auto module = json.find("module");
          const auto command = json.find("command");
          if (module == json.end() || command == json.end()) {
            AERROR << "Truncated module command.";
            return;
          }
          ExecuteComponentCommand(config_.modules(), *module, *command);
        });

    // HMI client asks for executing hardware command.
    websocket_->RegisterMessageHandler(
        "ExecuteHardwareCommand",
        [this](const Json &json, WebSocketHandler::Connection *conn) {
          // json should contain
          // {hardware: "hardware_name", command: "command_name"}.
          const auto hardware = json.find("hardware");
          const auto command = json.find("command");
          if (hardware == json.end() || command == json.end()) {
            AERROR << "Truncated hardware command.";
            return;
          }
          ExecuteComponentCommand(config_.hardware(), *hardware, *command);
        });

    // HMI client asks for changing driving mode.
    websocket_->RegisterMessageHandler(
        "ChangeDrivingMode",
        [this](const Json &json, WebSocketHandler::Connection *conn) {
          // json should contain {new_mode: "DrivingModeName"}.
          // DrivingModeName should be one of canbus::Chassis::DrivingMode.
          // For now it is either COMPLETE_MANUAL or COMPLETE_AUTO_DRIVE.
          const auto new_mode = json.find("new_mode");
          if (new_mode == json.end()) {
            AERROR << "Truncated ChangeDrivingMode request.";
            return;
          }
          ChangeDrivingModeTo(*new_mode);
        });

    // HMI client asks for changing map.
    websocket_->RegisterMessageHandler(
        "ChangeMap",
        [this](const Json &json, WebSocketHandler::Connection *conn) {
          // json should contain {new_map: "MapName"}.
          // MapName should be a key of config_.available_maps.
          const auto new_map = json.find("new_map");
          if (new_map == json.end()) {
            AERROR << "Truncated ChangeMap request.";
            return;
          }
          ChangeMapTo(*new_map);
        });

    // HMI client asks for changing vehicle.
    websocket_->RegisterMessageHandler(
        "ChangeVehicle",
        [this](const Json &json, WebSocketHandler::Connection *conn) {
          // json should contain {new_vehicle: "VehicleName"}.
          // VehicleName should be a key of config_.available_vehicles.
          const auto new_vehicle = json.find("new_vehicle");
          if (new_vehicle == json.end()) {
            AERROR << "Truncated ChangeVehicle request.";
            return;
          }
          ChangeVehicleTo(*new_vehicle);
        });
  }
}

void HMI::Start() {
}

void HMI::BroadcastHMIStatus() const {
  // In unit tests, we may leave websocket_ as NULL and skip broadcasting.
  if (websocket_) {
    websocket_->BroadcastData(ProtoToTypedJson("HMIStatus", status_));
  }
}

int HMI::ExecuteComponentCommand(const Map<std::string, Component> &components,
                                 const std::string &component_name,
                                 const std::string &command_name) {
  const auto component = components.find(component_name);
  if (component == components.end()) {
    AERROR << "Cannot find component with name " << component_name;
    return -1;
  }
  const auto &supported_commands = component->second.supported_commands();
  const auto cmd = supported_commands.find(command_name);
  if (cmd == supported_commands.end()) {
    AERROR << "Cannot find command with name " << command_name
           << " for component " << component_name;
    return -1;
  }
  AINFO << "Execute system command: " << cmd->second;
  int ret = std::system(cmd->second.c_str());

  AERROR_IF(ret != 0) << "Command returns " << ret << ": " << cmd->second;
  return ret;
}

void HMI::ChangeDrivingModeTo(const std::string &new_mode) {
  Chassis::DrivingMode mode;
  if (!Chassis::DrivingMode_Parse(new_mode, &mode)) {
    AERROR << "Unknown driving mode " << new_mode;
    return;
  }

  auto driving_action = DrivingAction::RESET;
  switch (mode) {
    case Chassis::COMPLETE_MANUAL:
      // Default driving action: RESET.
      break;
    case Chassis::COMPLETE_AUTO_DRIVE:
      driving_action = DrivingAction::START;
      break;
    default:
      AERROR << "Unknown action to change driving mode to " << new_mode;
      return;
  }

  control::PadMessage pad;
  pad.set_action(driving_action);
  AdapterManager::FillPadHeader("HMI", &pad);
  AdapterManager::PublishPad(pad);
}

void HMI::ChangeMapTo(const std::string &map_name) {
  const auto iter = config_.available_maps().find(map_name);
  if (iter == config_.available_maps().end()) {
    AERROR << "Unknown map " << map_name;
    return;
  }
  // Append new map_dir flag to global flagfile.
  std::ofstream fout(FLAGS_global_flagfile, std::ios_base::app);
  CHECK(fout) << "Fail to open " << FLAGS_global_flagfile;
  fout << "\n--map_dir=" << iter->second << std::endl;

  // TODO(xiaoxq): Reset all modules.
  status_.set_current_map(map_name);
}

void HMI::ChangeVehicleTo(const std::string &vehicle_name) {
  const auto iter = config_.available_vehicles().find(vehicle_name);
  if (iter == config_.available_vehicles().end()) {
    AERROR << "Unknown vehicle " << vehicle_name;
    return;
  }

  // TODO(xiaoxq): Copy vehicle params to target position, and reset all modules
  // and hardware.
  status_.set_current_vehicle(vehicle_name);
}

}  // namespace dreamview
}  // namespace apollo
