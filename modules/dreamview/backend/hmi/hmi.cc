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

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/json_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/point_cloud/point_cloud_updater.h"

namespace apollo {
namespace dreamview {

using apollo::common::util::JsonUtil;
using Json = WebSocketHandler::Json;

HMI::HMI(WebSocketHandler* websocket, MapService* map_service,
         DataCollectionMonitor* data_collection_monitor)
    : hmi_worker_(new HMIWorker()),
      monitor_log_buffer_(apollo::common::monitor::MonitorMessageItem::HMI),
      websocket_(websocket),
      map_service_(map_service),
      data_collection_monitor_(data_collection_monitor) {
  if (websocket_) {
    RegisterMessageHandlers();
  }
}

void HMI::Start() { hmi_worker_->Start(); }

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
          CHECK(map_service_->ReloadMap(true))
              << "Failed to load new simulation map: " << value;
        } else if (hmi_action == HMIAction::CHANGE_VEHICLE) {
          // Reload lidar params for point cloud service.
          PointCloudUpdater::LoadLidarHeight(FLAGS_lidar_height_yaml);
          SendVehicleParam();
          if (data_collection_monitor_->IsEnabled()) {
            data_collection_monitor_->Restart();
          }
        } else if (hmi_action == HMIAction::CHANGE_MODE) {
          static constexpr char kCalibrationMode[] = "Vehicle Calibration";
          if (value == kCalibrationMode) {
            data_collection_monitor_->Start();
          } else {
            data_collection_monitor_->Stop();
          }
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

}  // namespace dreamview
}  // namespace apollo
