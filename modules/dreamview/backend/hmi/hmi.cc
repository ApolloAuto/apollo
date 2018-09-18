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
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/json_util.h"
#include "modules/common/util/map_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/hmi/hmi_worker.h"

namespace apollo {
namespace dreamview {

using apollo::canbus::Chassis;
using apollo::common::VehicleConfigHelper;
using apollo::common::time::Clock;
using apollo::common::util::JsonUtil;
using Json = WebSocketHandler::Json;
using RLock = boost::shared_lock<boost::shared_mutex>;

HMI::HMI(WebSocketHandler *websocket, MapService *map_service)
    : node_(cybertron::CreateNode("hmi")),
      websocket_(websocket),
      map_service_(map_service),
      monitor_log_buffer_(apollo::common::monitor::MonitorMessageItem::HMI) {
  // Register websocket message handlers.
  if (websocket_) {
    RegisterMessageHandlers();
    StartBroadcastHMIStatusThread();
  }

  // TODO(linvivian): make sure the writers are thread-safe
  audio_capture_writer_ =
      node_->CreateWriter<AudioCapture>(FLAGS_audio_capture_topic);

  HMIWorker::Instance()->Init(node_);
}

void HMI::RegisterMessageHandlers() {
  // Send current config and status to new HMI client.
  websocket_->RegisterConnectionReadyHandler(
      [this](WebSocketHandler::Connection *conn) {
        const auto &config = HMIWorker::Instance()->GetConfig();
        websocket_->SendData(
            conn, JsonUtil::ProtoToTypedJson("HMIConfig", config).dump());
        websocket_->SendData(
            conn, JsonUtil::ProtoToTypedJson("HMIStatus",
                                             HMIWorker::Instance()->GetStatus())
                      .dump());

        SendVehicleParam(conn);
      });

  // HMI client sends audio data, publish to AudioCapture topic.
  websocket_->RegisterMessageHandler(
      "AudioPiece",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {data: "<base64 encoded audio/wav piece>"}.
        std::string data;
        if (JsonUtil::GetStringFromJson(json, "data", &data)) {
          AudioCapture audio;
          audio.set_connection_id(reinterpret_cast<uint64_t>(conn));
          audio.set_wav_stream(apollo::common::util::DecodeBase64(data));
          audio_capture_writer_->Write(std::make_shared<AudioCapture>(audio));
        } else {
          AERROR << "Truncated audio piece.";
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
          HMIWorker::Instance()->RunModuleCommand(module, command);
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
          HMIWorker::Instance()->RunToolCommand(tool, command);
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
          HMIWorker::Instance()->RunModeCommand(command);
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
  HMIWorker::Instance()->RegisterChangeMapHandler(
      [this](const std::string &new_map) {
        // Reload simulation map after changing map.
        CHECK(map_service_->ReloadMap(true))
            << "Failed to load new simulation map: " << new_map;
        // And then broadcast new HMIStatus to all clients.
        DeferredBroadcastHMIStatus();
      });
  websocket_->RegisterMessageHandler(
      "ChangeMap",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {new_map: "MapName"}.
        // MapName should be a key of config_.available_maps.
        std::string new_map;
        if (JsonUtil::GetStringFromJson(json, "new_map", &new_map)) {
          HMIWorker::Instance()->ChangeToMap(new_map);
        } else {
          AERROR << "Truncated ChangeMap request.";
        }
      });

  // HMI client asks for changing vehicle.
  HMIWorker::Instance()->RegisterChangeVehicleHandler(
      [this](const std::string &new_vehicle) {
        // Broadcast new HMIStatus and VehicleParam.
        DeferredBroadcastHMIStatus();
        SendVehicleParam();
      });
  websocket_->RegisterMessageHandler(
      "ChangeVehicle",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {new_vehicle: "VehicleName"}.
        // VehicleName should be a key of config_.available_vehicles.
        std::string new_vehicle;
        if (JsonUtil::GetStringFromJson(json, "new_vehicle", &new_vehicle)) {
          HMIWorker::Instance()->ChangeToVehicle(new_vehicle);
        } else {
          AERROR << "Truncated ChangeVehicle request.";
        }
      });

  // HMI client asks for changing mode.
  HMIWorker::Instance()->RegisterChangeModeHandler(
      [this](const std::string &new_mode) {
        // Broadcast new HMIStatus.
        DeferredBroadcastHMIStatus();
      });
  websocket_->RegisterMessageHandler(
      "ChangeMode",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {new_mode: "ModeName"}.
        // ModeName should be a key of config_.modes.
        std::string new_mode;
        if (JsonUtil::GetStringFromJson(json, "new_mode", &new_mode)) {
          HMIWorker::Instance()->ChangeToMode(new_mode);
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
        std::vector<std::string> event_types;
        if (JsonUtil::GetNumberFromJson(json, "event_time_ms",
                                        &event_time_ms) &&
            JsonUtil::GetStringFromJson(json, "event_msg", &event_msg) &&
            JsonUtil::GetStringVectorFromJson(json, "event_type",
                                              &event_types)) {
          HMIWorker::SubmitDriveEvent(event_time_ms, event_msg, event_types);
          monitor_log_buffer_.INFO("Drive event added.");
        } else {
          AERROR << "Truncated SubmitDriveEvent request.";
          monitor_log_buffer_.WARN("Failed to submit a drive event.");
        }
      });

  // Received new system status, broadcast to clients.
  system_status_reader_ = node_->CreateReader<monitor::SystemStatus>(
      FLAGS_system_status_topic,
      [this](const std::shared_ptr<monitor::SystemStatus> &system_status) {
        ADEBUG << "Received system status: run system_status callback.";
        if (Clock::NowInSeconds() - system_status->header().timestamp_sec() <
            FLAGS_system_status_lifetime_seconds) {
          HMIWorker::Instance()->UpdateSystemStatus(*system_status);
          DeferredBroadcastHMIStatus();
        }
      });

  // Received Chassis, trigger action if there is high beam signal.
  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis> &chassis) {
        if (Clock::NowInSeconds() - chassis->header().timestamp_sec() <
            FLAGS_system_status_lifetime_seconds) {
          if (chassis->signal().high_beam()) {
            const bool ret = HMIWorker::Instance()->Trigger(
                HMIWorker::Instance()->GetConfig().chassis_high_beam_action());
            AERROR_IF(!ret) << "Failed to execute high_beam action.";
          }
        }
      });
}

void HMI::StartBroadcastHMIStatusThread() {
  constexpr int kMinBroadcastIntervalMs = 200;
  broadcast_hmi_status_thread_.reset(new std::thread([this]() {
    while (true) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(kMinBroadcastIntervalMs));

      {
        std::lock_guard<std::mutex> lock(need_broadcast_mutex_);
        if (!need_broadcast_) {
          continue;
        }
        // Reset to false.
        need_broadcast_ = false;
      }

      // Get a copy of status.
      const auto status = HMIWorker::Instance()->GetStatus();
      websocket_->BroadcastData(
          JsonUtil::ProtoToTypedJson("HMIStatus", status).dump());

      // Broadcast messages.
      if (status.current_map().empty()) {
        monitor_log_buffer_.WARN("You haven't selected a map yet!");
      }
      if (status.current_vehicle().empty()) {
        monitor_log_buffer_.WARN("You haven't selected a vehicle yet!");
      }
    }
  }));
}

void HMI::DeferredBroadcastHMIStatus() {
  std::lock_guard<std::mutex> lock(need_broadcast_mutex_);
  need_broadcast_ = true;
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
