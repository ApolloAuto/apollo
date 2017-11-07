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
#include <vector>

#include "gflags/gflags.h"
#include "google/protobuf/util/json_util.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/monitor/proto/system_status.pb.h"

DEFINE_string(global_flagfile, "modules/common/data/global_flagfile.txt",
              "Global flagfile shared by all modules.");

DEFINE_string(map_data_path, "modules/map/data", "Path to map data.");

DEFINE_string(vehicle_data_path, "modules/calibration/data",
              "Path to vehicle data.");

namespace apollo {
namespace dreamview {
namespace {

using apollo::canbus::Chassis;
using apollo::common::adapter::AdapterManager;
using apollo::common::util::FindOrNull;
using apollo::common::util::StringTokenizer;
using apollo::control::DrivingAction;
using google::protobuf::Map;
using Json = WebSocketHandler::Json;

std::string ProtoToTypedJson(const std::string &json_type,
                             const google::protobuf::Message &proto) {
  std::string json_string;
  google::protobuf::util::MessageToJsonString(proto, &json_string);

  Json json_obj;
  json_obj["type"] = json_type;
  json_obj["data"] = Json::parse(json_string);
  return json_obj.dump();
}

// Convert a string to be title-like. E.g.: "hello_world" -> "Hello World".
std::string TitleCase(const std::string &origin,
                      const std::string &delimiter = "_") {
  std::vector<std::string> parts = StringTokenizer::Split(origin, delimiter);
  for (auto &part : parts) {
    if (!part.empty()) {
      // Upper case the first char.
      part[0] = toupper(part[0]);
    }
  }

  return apollo::common::util::StrCat(apollo::common::util::PrintIter(parts));
}

// List subdirs and return a dict of {subdir_title: subdir_path}.
Map<std::string, std::string> ListDirAsDict(const std::string &dir) {
  Map<std::string, std::string> result;
  const auto subdirs = apollo::common::util::ListSubDirectories(dir);
  for (const auto &subdir : subdirs) {
    const auto subdir_title = TitleCase(subdir);
    const auto subdir_path = apollo::common::util::StrCat(dir, "/", subdir);
    result.insert({subdir_title, subdir_path});
  }
  return result;
}

}  // namespace

HMI::HMI(WebSocketHandler *websocket) : websocket_(websocket) {
  CHECK(common::util::GetProtoFromFile(FLAGS_hmi_config_filename, &config_))
      << "Unable to parse HMI config file " << FLAGS_hmi_config_filename;
  // If the module path doesn't exist, remove it from list.
  auto *modules = config_.mutable_modules();
  for (auto iter = modules->begin(); iter != modules->end();) {
    const auto &conf = iter->second;
    if (conf.has_path() && !common::util::PathExists(conf.path())) {
      iter = modules->erase(iter);
    } else {
      ++iter;
    }
  }

  // Get available maps and vehicles by listing data directory.
  *config_.mutable_available_maps() = ListDirAsDict(FLAGS_map_data_path);
  *config_.mutable_available_vehicles() =
      ListDirAsDict(FLAGS_vehicle_data_path);
  ADEBUG << "Loaded HMI config: " << config_.DebugString();

  // Register websocket message handlers.
  if (websocket_) {
    RegisterMessageHandlers();
  }
}

void HMI::RegisterMessageHandlers() {
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
        // json should contain {module: "module_name", command: "command_name"}.
        // If module_name is "all", then run the command on all modules.
        const auto module = json.find("module");
        const auto command = json.find("command");
        if (module == json.end() || command == json.end()) {
          AERROR << "Truncated module command.";
          return;
        }
        if (*module == "all") {
          RunCommandOnAllModules(*command);
        } else {
          RunComponentCommand(config_.modules(), *module, *command);
        }
      });

  // HMI client asks for executing tool command.
  websocket_->RegisterMessageHandler(
      "ExecuteToolCommand",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {tool: "tool_name", command: "command_name"}.
        const auto tool = json.find("tool");
        const auto command = json.find("command");
        if (tool == json.end() || command == json.end()) {
          AERROR << "Truncated tool command.";
          return;
        }
        RunComponentCommand(config_.tools(), *tool, *command);
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

  // Received new system status, broadcast to clients.
  AdapterManager::AddSystemStatusCallback(
      [this](const monitor::SystemStatus &system_status) {
        *status_.mutable_system_status() = system_status;
        BroadcastHMIStatus();
      });
}

void HMI::BroadcastHMIStatus() const {
  // In unit tests, we may leave websocket_ as NULL and skip broadcasting.
  if (websocket_) {
    websocket_->BroadcastData(ProtoToTypedJson("HMIStatus", status_));
  }
}

int HMI::RunComponentCommand(const Map<std::string, Component> &components,
                             const std::string &component_name,
                             const std::string &command_name) {
  const auto *component = FindOrNull(components, component_name);
  if (component == nullptr) {
    AERROR << "Cannot find component " << component_name;
    return -1;
  }
  const auto *cmd = FindOrNull(component->supported_commands(), command_name);
  if (cmd == nullptr) {
    AERROR << "Cannot find command " << component_name << "." << command_name;
    return -1;
  }
  ADEBUG << "Execute system command: " << *cmd;
  const int ret = std::system(cmd->c_str());

  AERROR_IF(ret != 0) << "Command returns " << ret << ": " << *cmd;
  return ret;
}

int HMI::RunCommandOnAllModules(const std::string &command_name) {
  int failed = 0;
  const auto &modules = config_.modules();
  for (const auto &module : modules) {
    const int ret = RunComponentCommand(modules, module.first, command_name);
    if (ret != 0) {
      ++failed;
    }
  }
  return failed;
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

  RunCommandOnAllModules("stop");
  status_.set_current_map(map_name);
  BroadcastHMIStatus();
}

void HMI::ChangeVehicleTo(const std::string &vehicle_name) {
  const auto iter = config_.available_vehicles().find(vehicle_name);
  if (iter == config_.available_vehicles().end()) {
    AERROR << "Unknown vehicle " << vehicle_name;
    return;
  }

  RunCommandOnAllModules("stop");
  status_.set_current_vehicle(vehicle_name);
  BroadcastHMIStatus();
}

}  // namespace dreamview
}  // namespace apollo
