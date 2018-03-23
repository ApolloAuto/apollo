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
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/json_util.h"
#include "modules/common/util/map_util.h"
#include "modules/dreamview/backend/hmi/hmi_worker.h"
#include "modules/monitor/proto/system_status.pb.h"

DEFINE_double(system_status_lifetime_seconds, 30,
              "Lifetime of a valid SystemStatus message.");

namespace apollo {
namespace dreamview {

using apollo::canbus::Chassis;
using apollo::common::VehicleConfigHelper;
using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;
using apollo::common::util::FindLinkedPtrOrNull;
using apollo::common::util::JsonUtil;
using apollo::common::util::make_unique;
using Json = WebSocketHandler::Json;
using RLock = boost::shared_lock<boost::shared_mutex>;

HMI::HMI(WebSocketHandler *websocket, MapService *map_service)
    : websocket_(websocket),
      map_service_(map_service),
      logger_(apollo::common::monitor::MonitorMessageItem::HMI) {
  // Register websocket message handlers.
  if (websocket_) {
    RegisterMessageHandlers();
  }
}

void HMI::RegisterMessageHandlers() {
  // Send current config and status to new HMI client.
  websocket_->RegisterConnectionReadyHandler(
      [this](WebSocketHandler::Connection *conn) {
        const auto &config = HMIWorker::instance()->GetConfig();
        websocket_->SendData(
            conn, JsonUtil::ProtoToTypedJson("HMIConfig", config).dump());

        {
          RLock rlock(HMIWorker::instance()->GetStatusMutex());
          const auto &status = HMIWorker::instance()->GetStatus();
          websocket_->SendData(
              conn, JsonUtil::ProtoToTypedJson("HMIStatus", status).dump());
        }
        SendVehicleParam(conn);
      });

  // HMI client sends voice data.
  websocket_->RegisterMessageHandler(
      "VoicePiece",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {data: "<base64 encoded audio/wav piece>"}.
        std::string data;
        if (JsonUtil::GetStringFromJson(json, "data", &data)) {
          VoiceDetectionRequest request;
          request.set_id(reinterpret_cast<uint64_t>(conn));
          request.set_wav_stream(apollo::common::util::Base64Decode(data));
          AdapterManager::PublishVoiceDetectionRequest(request);
        } else {
          AERROR << "Truncated voice piece.";
        }
      });

  // HMI client asks for executing module command.
  websocket_->RegisterMessageHandler(
      "ExecuteModuleCommand",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {module: "module_name", command: "command_name"}.
        // If module_name is "all", then run the command on all modules.
        std::string module;
        std::string command;
        if (JsonUtil::GetStringFromJson(json, "module", &module) &&
            JsonUtil::GetStringFromJson(json, "command", &command)) {
          HMIWorker::instance()->RunModuleCommand(module, command);
        } else {
          AERROR << "Truncated module command.";
        }
      });

  // HMI client asks for executing tool command.
  websocket_->RegisterMessageHandler(
      "ExecuteToolCommand",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {tool: "tool_name", command: "command_name"}.
        std::string tool;
        std::string command;
        if (JsonUtil::GetStringFromJson(json, "tool", &tool) &&
            JsonUtil::GetStringFromJson(json, "command", &command)) {
          HMIWorker::instance()->RunToolCommand(tool, command);
        } else {
          AERROR << "Truncated tool command.";
        }
      });

  // HMI client asks for executing mode command.
  websocket_->RegisterMessageHandler(
      "ExecuteModeCommand",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {command: "command_name"}.
        // Supported commands are: "start", "stop".
        std::string command;
        if (JsonUtil::GetStringFromJson(json, "command", &command)) {
          HMIWorker::instance()->RunModeCommand(command);
        } else {
          AERROR << "Truncated mode command.";
        }
      });

  // HMI client asks for changing driving mode.
  websocket_->RegisterMessageHandler(
      "ChangeDrivingMode",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {new_mode: "DrivingModeName"}.
        // DrivingModeName should be one of canbus::Chassis::DrivingMode.
        // For now it is either COMPLETE_MANUAL or COMPLETE_AUTO_DRIVE.
        std::string new_mode;
        if (JsonUtil::GetStringFromJson(json, "new_mode", &new_mode)) {
          Chassis::DrivingMode mode;
          if (Chassis::DrivingMode_Parse(new_mode, &mode)) {
            HMIWorker::ChangeToDrivingMode(mode);
          } else {
            AERROR << "Unknown driving mode " << new_mode;
          }
        } else {
          AERROR << "Truncated ChangeDrivingMode request.";
        }
      });

  // HMI client asks for changing map.
  websocket_->RegisterMessageHandler(
      "ChangeMap",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {new_map: "MapName"}.
        // MapName should be a key of config_.available_maps.
        std::string new_map;
        if (JsonUtil::GetStringFromJson(json, "new_map", &new_map)) {
          HMIWorker::instance()->ChangeToMap(new_map, map_service_);
          BroadcastHMIStatus();
        } else {
          AERROR << "Truncated ChangeMap request.";
        }
      });

  // HMI client asks for changing vehicle.
  websocket_->RegisterMessageHandler(
      "ChangeVehicle",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {new_vehicle: "VehicleName"}.
        // VehicleName should be a key of config_.available_vehicles.
        std::string new_vehicle;
        if (JsonUtil::GetStringFromJson(json, "new_vehicle", &new_vehicle)) {
          HMIWorker::instance()->ChangeToVehicle(new_vehicle);
          BroadcastHMIStatus();
          // Broadcast new VehicleParam.
          SendVehicleParam();
        } else {
          AERROR << "Truncated ChangeVehicle request.";
        }
      });

  // HMI client asks for changing mode.
  websocket_->RegisterMessageHandler(
      "ChangeMode",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {new_mode: "ModeName"}.
        // ModeName should be a key of config_.modes.
        std::string new_mode;
        if (JsonUtil::GetStringFromJson(json, "new_mode", &new_mode)) {
          HMIWorker::instance()->ChangeToMode(new_mode);
          BroadcastHMIStatus();
        } else {
          AERROR << "Truncated ChangeMode request.";
        }
      });

  // HMI client asks for adding new DriveEvent.
  websocket_->RegisterMessageHandler(
      "SubmitDriveEvent",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain event_time_ms and event_msg.
        uint64_t event_time_ms;
        std::string event_msg;
        apollo::common::monitor::MonitorLogBuffer log_buffer(&logger_);
        if (JsonUtil::GetNumberFromJson(json, "event_time_ms",
                                        &event_time_ms) &&
            JsonUtil::GetStringFromJson(json, "event_msg", &event_msg)) {
          HMIWorker::SubmitDriveEvent(event_time_ms, event_msg);
          log_buffer.INFO("Drive event added.");
        } else {
          AERROR << "Truncated SubmitDriveEvent request.";
          log_buffer.WARN("Failed to submit a drive event.");
        }
      });

  // Received new system status, broadcast to clients.
  AdapterManager::AddSystemStatusCallback(
      [this](const monitor::SystemStatus &system_status) {
        if (Clock::NowInSeconds() - system_status.header().timestamp_sec() <
            FLAGS_system_status_lifetime_seconds) {
          HMIWorker::instance()->UpdateSystemStatus(system_status);
          BroadcastHMIStatus();
        }
      });

  // Received VoiceDetection response.
  AdapterManager::AddVoiceDetectionResponseCallback(
      [this](const VoiceDetectionResponse &response) {
        apollo::common::monitor::MonitorLogBuffer log_buffer(&logger_);
        log_buffer.INFO() << "Triggered action by voice: "
                          << HMIAction_Name(response.action());
        HMIWorker::instance()->Trigger(response.action());
      });
}

void HMI::BroadcastHMIStatus() {
  // In unit tests, we may leave websocket_ as NULL and skip broadcasting.
  RLock rlock(HMIWorker::instance()->GetStatusMutex());
  const auto &status = HMIWorker::instance()->GetStatus();
  if (websocket_) {
    websocket_->BroadcastData(
        JsonUtil::ProtoToTypedJson("HMIStatus", status).dump());
  }

  // Broadcast messages.
  apollo::common::monitor::MonitorLogBuffer log_buffer(&logger_);
  if (status.current_map().empty()) {
    log_buffer.WARN("You haven't select map yet!");
  }
  if (status.current_vehicle().empty()) {
    log_buffer.WARN("You haven't select vehicle yet!");
  }
}

void HMI::SendVehicleParam(WebSocketHandler::Connection *conn) {
  if (websocket_ == nullptr) {
    return;
  }

  const auto json_str =
      JsonUtil::ProtoToTypedJson(
          "VehicleParam", VehicleConfigHelper::GetConfig().vehicle_param())
          .dump();
  if (conn != nullptr) {
    websocket_->SendData(conn, json_str);
  } else {
    websocket_->BroadcastData(json_str);
  }
}

}  // namespace dreamview
}  // namespace apollo
