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
#include "google/protobuf/util/json_util.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

namespace apollo {
namespace dreamview {

using apollo::canbus::Chassis;
using apollo::common::adapter::AdapterManager;
using apollo::control::DrivingAction;
using google::protobuf::RepeatedPtrField;
using Json = WebSocketHandler::Json;

namespace {

std::string ProtoToTypedJsonString(const std::string &json_type,
                                   const google::protobuf::Message &proto) {
  std::string json_string;
  google::protobuf::util::MessageToJsonString(proto, &json_string);

  Json json_obj;
  json_obj["type"] = json_type;
  json_obj["data"] = Json::parse(json_string);
  return json_obj.dump();
}

// NamedValue is a proto that has "string name;" field.
// We'll find the first value with the given name, or NULL if none exists.
template <class NamedValue>
const NamedValue *FindByName(const RepeatedPtrField<NamedValue> &named_values,
                             const std::string &name) {
  for (const auto &value : named_values) {
    if (value.name() == name) {
      return &value;
    }
  }
  return nullptr;
}

void ChangeDrivingModeTo(const std::string &target_mode) {
  Chassis::DrivingMode mode;
  if (!Chassis::DrivingMode_Parse(target_mode, &mode)) {
    AERROR << "Unknown target driving mode " << target_mode;
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
      AERROR << "Unknown action to change driving mode to " << target_mode;
      return;
  }

  control::PadMessage pad;
  pad.set_action(driving_action);
  AdapterManager::FillPadHeader("HMI", &pad);
  AdapterManager::PublishPad(pad);
}

}  // namespace

HMI::HMI(WebSocketHandler *websocket) : websocket_(websocket) {
  CHECK(common::util::GetProtoFromFile(FLAGS_hmi_config_filename, &config_))
      << "Unable to parse HMI config file " << FLAGS_hmi_config_filename;

  // Init status of modules and hardware.
  for (const auto &module : config_.modules()) {
    status_.add_modules()->set_name(module.name());
  }
  for (const auto &hardware : config_.hardware()) {
    status_.add_hardware()->set_name(hardware.name());
  }

  // Register websocket message handlers.
  if (websocket_) {
    // Newly opened HMI client retrieves current status.
    websocket_->RegisterMessageHandler(
        "RetrieveHMIStatus",
        [this](const Json &json, WebSocketHandler::Connection *conn) {
          SendHMIStatus(conn);
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
          // json should contain {target_mode: "DrivingModeName"}.
          // DrivingModeName should be one of canbus::Chassis::DrivingMode.
          // For now it is either COMPLETE_MANUAL or COMPLETE_AUTO_DRIVE.
          const auto target_mode_value = json.find("target_mode");
          if (target_mode_value == json.end()) {
            AERROR << "Truncated ChangeDrivingMode request.";
            return;
          }
          ChangeDrivingModeTo(*target_mode_value);
        });
  }
}

void HMI::Start() {
  AdapterManager::AddHMIStatusCallback(&HMI::OnHMIStatus, this);
}

void HMI::OnHMIStatus(const HMIStatus &hmi_status) {
  // Note that we only merged status of modules and hardware, which matches the
  // hmi_status_helper.
  for (const auto &new_status : hmi_status.modules()) {
    auto *old_status = GetModuleStatus(new_status.name());
    if (old_status) {
      old_status->MergeFrom(new_status);
    } else {
      AERROR << "Updating HMIStatus, unknown module " << new_status.name();
    }
  }
  for (const auto &new_status : hmi_status.hardware()) {
    auto *old_status = GetHardwareStatus(new_status.name());
    if (old_status) {
      old_status->MergeFrom(new_status);
    } else {
      AERROR << "Updating HMIStatus, unknown hardware " << new_status.name();
    }
  }

  BroadcastHMIStatus();
}

void HMI::BroadcastHMIStatus() const {
  // In unit tests, we may leave websocket_ as NULL and skip broadcasting.
  if (websocket_) {
    websocket_->BroadcastData(ProtoToTypedJsonString("HMIStatus", status_));
  }
}

void HMI::SendHMIStatus(WebSocketHandler::Connection *conn) const {
  CHECK(websocket_);
  websocket_->SendData(conn, ProtoToTypedJsonString("HMIStatus", status_));
}

ModuleStatus* HMI::GetModuleStatus(const std::string &module_name) {
  for (auto &module_status : *status_.mutable_modules()) {
    if (module_status.name() == module_name) {
      return &module_status;
    }
  }
  return nullptr;
}

HardwareStatus* HMI::GetHardwareStatus(const std::string &hardware_name) {
  for (auto &hardware_status : *status_.mutable_hardware()) {
    if (hardware_status.name() == hardware_name) {
      return &hardware_status;
    }
  }
  return nullptr;
}

int HMI::ExecuteComponentCommand(const RepeatedPtrField<Component> &components,
                                 const std::string &component_name,
                                 const std::string &command_name) {
  const auto* component = FindByName(components, component_name);
  if (component == nullptr) {
    AERROR << "Cannot find component with name " << component_name;
    return -1;
  }
  const auto* cmd = FindByName(component->supported_commands(), command_name);
  if (cmd == nullptr) {
    AERROR << "Cannot find command with name " << command_name
           << " for component " << component_name;
    return -1;
  }
  const std::string cmd_string = common::util::StrCat(
      common::util::PrintIter(cmd->command()));
  AINFO << "Execute system command: " << cmd_string;
  int ret = std::system(cmd_string.c_str());

  AERROR_IF(ret != 0) << "Command returns " << ret << ": " << cmd_string;
  return ret;
}

}  // namespace dreamview
}  // namespace apollo
