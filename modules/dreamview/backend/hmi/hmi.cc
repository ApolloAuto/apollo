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

#include <string>
#include <vector>

#include "google/protobuf/util/json_util.h"

#include "modules/dreamview/proto/preprocess_table.pb.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/json_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/common/fuel_monitor/fuel_monitor_manager.h"
#include "modules/dreamview/backend/point_cloud/point_cloud_updater.h"

namespace apollo {
namespace dreamview {

using apollo::common::util::JsonUtil;
using apollo::cyber::common::SetProtoToASCIIFile;
using google::protobuf::util::JsonStringToMessage;
using Json = WebSocketHandler::Json;

HMI::HMI(WebSocketHandler* websocket, MapService* map_service)
    : hmi_worker_(new HMIWorker()),
      monitor_log_buffer_(apollo::common::monitor::MonitorMessageItem::HMI),
      websocket_(websocket),
      map_service_(map_service) {
  if (websocket_) {
    RegisterMessageHandlers();
  }
}

void HMI::Start(DvCallback callback_api) { hmi_worker_->Start(callback_api); }

void HMI::Stop() { hmi_worker_->Stop(); }

void HMI::RegisterMessageHandlers() {
  // Broadcast HMIStatus to clients when status changed.
  hmi_worker_->RegisterStatusUpdateHandler(
      [this](const bool status_changed, HMIStatus* status) {
        if (!status_changed) {
          // Status doesn't change, skip broadcasting.
          return;
        }
        websocket_->BroadcastData(
            JsonUtil::ProtoToTypedJson("HMIStatus", *status).dump());
        if (status->current_map().empty()) {
          monitor_log_buffer_.WARN("You haven't selected a map yet!");
        }
        if (status->current_vehicle().empty()) {
          monitor_log_buffer_.WARN("You haven't selected a vehicle yet!");
        }
      });

  // Send current status and vehicle param to newly joined client.
  websocket_->RegisterConnectionReadyHandler(
      [this](WebSocketHandler::Connection* conn) {
        SendStatus(conn);
        SendVehicleParam(conn);
      });

  websocket_->RegisterMessageHandler(
      "HMIAction",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        // Run HMIWorker::Trigger(action) if json is {action: "<action>"}
        // Run HMIWorker::Trigger(action, value) if "value" field is provided.
        std::string action;
        if (!JsonUtil::GetString(json, "action", &action)) {
          AERROR << "Truncated HMIAction request.";
          return;
        }
        HMIAction hmi_action;
        if (!HMIAction_Parse(action, &hmi_action)) {
          AERROR << "Invalid HMIAction string: " << action;
          return;
        }
        std::string value;
        if (JsonUtil::GetString(json, "value", &value)) {
          hmi_worker_->Trigger(hmi_action, value);
        } else {
          hmi_worker_->Trigger(hmi_action);
        }

        // Extra works for current Dreamview.
        if (hmi_action == HMIAction::CHANGE_MAP) {
          // Reload simulation map after changing map.
          ACHECK(map_service_->ReloadMap(true))
              << "Failed to load new simulation map: " << value;
        } else if (hmi_action == HMIAction::CHANGE_VEHICLE) {
          // Reload lidar params for point cloud service.
          PointCloudUpdater::LoadLidarHeight(FLAGS_lidar_height_yaml);
          SendVehicleParam();
        }
      });

  // HMI client asks for adding new AudioEvent.
  websocket_->RegisterMessageHandler(
      "SubmitAudioEvent",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        // json should contain event_time_ms, obstacle_id, audio_type,
        // moving_result, audio_direction and is_siren_on.
        uint64_t event_time_ms;
        int obstacle_id;
        int audio_type;
        int moving_result;
        int audio_direction;
        bool is_siren_on;
        if (JsonUtil::GetNumber(json, "event_time_ms", &event_time_ms) &&
            JsonUtil::GetNumber(json, "obstacle_id", &obstacle_id) &&
            JsonUtil::GetNumber(json, "audio_type", &audio_type) &&
            JsonUtil::GetNumber(json, "moving_result", &moving_result) &&
            JsonUtil::GetNumber(json, "audio_direction", &audio_direction) &&
            JsonUtil::GetBoolean(json, "is_siren_on", &is_siren_on)) {
          hmi_worker_->SubmitAudioEvent(event_time_ms, obstacle_id, audio_type,
                                        moving_result, audio_direction,
                                        is_siren_on);
          monitor_log_buffer_.INFO("Audio event added.");
        } else {
          AERROR << "Truncated SubmitAudioEvent request.";
          monitor_log_buffer_.WARN("Failed to submit an audio event.");
        }
      });

  // HMI client asks for adding new DriveEvent.
  websocket_->RegisterMessageHandler(
      "SubmitDriveEvent",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        // json should contain event_time_ms and event_msg.
        uint64_t event_time_ms;
        std::string event_msg;
        std::vector<std::string> event_types;
        bool is_reportable;
        if (JsonUtil::GetNumber(json, "event_time_ms", &event_time_ms) &&
            JsonUtil::GetString(json, "event_msg", &event_msg) &&
            JsonUtil::GetStringVector(json, "event_type", &event_types) &&
            JsonUtil::GetBoolean(json, "is_reportable", &is_reportable)) {
          hmi_worker_->SubmitDriveEvent(event_time_ms, event_msg, event_types,
                                        is_reportable);
          monitor_log_buffer_.INFO("Drive event added.");
        } else {
          AERROR << "Truncated SubmitDriveEvent request.";
          monitor_log_buffer_.WARN("Failed to submit a drive event.");
        }
      });

  websocket_->RegisterMessageHandler(
      "HMIStatus",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        SendStatus(conn);
      });

  websocket_->RegisterMessageHandler(
      "SensorCalibrationPreprocess",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        // json should contain type and data.
        std::string current_mode = hmi_worker_->GetStatus().current_mode();
        std::string task_type;
        if (current_mode == FLAGS_lidar_calibration_mode) {
          task_type = "lidar_to_gnss";
        } else if (current_mode == FLAGS_camera_calibration_mode) {
          task_type = "camera_to_lidar";
        } else {
          AERROR << "Unsupported mode:" << current_mode;
          return;
        }

        const auto iter = json.find("data");
        if (iter == json.end()) {
          AERROR << "The json has no such key: data";
          return;
        }
        PreprocessTable preprocess_table;
        if (!JsonStringToMessage(json["data"].dump(), &preprocess_table).ok()) {
          AERROR
              << "Failed to get user configuration: invalid preprocess table."
              << json.dump();
        }

        // Gernerate user-specified configuration and run the preprocess script
        std::string output_file =
            absl::StrCat("/apollo/modules/tools/sensor_calibration/config/",
                         task_type, "_user.config");
        if (!SetProtoToASCIIFile(preprocess_table, output_file)) {
          AERROR << "Failed to generate user configuration file";
        }
        hmi_worker_->SensorCalibrationPreprocess(task_type);
      });

  websocket_->RegisterMessageHandler(
      "VehicleCalibrationPreprocess",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        hmi_worker_->VehicleCalibrationPreprocess();
      });
}

void HMI::SendVehicleParam(WebSocketHandler::Connection* conn) {
  if (websocket_ == nullptr) {
    return;
  }

  const auto& vehicle_param =
      apollo::common::VehicleConfigHelper::GetConfig().vehicle_param();
  const std::string json_str =
      JsonUtil::ProtoToTypedJson("VehicleParam", vehicle_param).dump();
  if (conn != nullptr) {
    websocket_->SendData(conn, json_str);
  } else {
    websocket_->BroadcastData(json_str);
  }
}

void HMI::SendStatus(WebSocketHandler::Connection* conn) {
  const auto status_json =
      JsonUtil::ProtoToTypedJson("HMIStatus", hmi_worker_->GetStatus());
  websocket_->SendData(conn, status_json.dump());
}

bool HMI::UpdateScenarioSetToStatus(const std::string& scenario_set_id,
                                    const std::string& scenario_set_name) {
  return hmi_worker_->UpdateScenarioSetToStatus(scenario_set_id,
                                                scenario_set_name);
}

bool HMI::UpdateDynamicModelToStatus(const std::string& dynamic_model_name) {
  return hmi_worker_->UpdateDynamicModelToStatus(dynamic_model_name);
}

bool HMI::UpdateRecordToStatus() { return hmi_worker_->LoadRecords(); }

bool HMI::UpdateVehicleToStatus() { return hmi_worker_->ReloadVehicles(); }

bool HMI::UpdateCameraChannelToStatus(const std::string& channel_name) {
  hmi_worker_->UpdateCameraSensorChannelToStatus(channel_name);
  return true;
}

bool HMI::UpdatePointChannelToStatus(const std::string& channel_name) {
  hmi_worker_->UpdatePointCloudChannelToStatus(channel_name);
  return true;
}
}  // namespace dreamview
}  // namespace apollo
