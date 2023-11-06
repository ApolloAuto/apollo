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

#include "modules/dreamview_plus/backend/hmi/hmi.h"

#include <string>
#include <vector>

#include "google/protobuf/util/json_util.h"

#include "modules/dreamview_plus/proto/preprocess_table.pb.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/json_util.h"
#include "modules/dreamview_plus/backend/common/dreamview_gflags.h"
#include "modules/dreamview_plus/backend/fuel_monitor/fuel_monitor_manager.h"
#include "modules/dreamview_plus/backend/point_cloud/point_cloud_updater.h"

namespace apollo {
namespace dreamview {

using apollo::common::util::JsonUtil;
using apollo::cyber::common::SetProtoToASCIIFile;
using google::protobuf::util::JsonStringToMessage;
using Json = WebSocketHandler::Json;

HMI::HMI(WebSocketHandler* websocket, MapService* map_service,
         WebSocketHandler* hmi_websocket)
    : hmi_worker_(new HMIWorker()),
      monitor_log_buffer_(apollo::common::monitor::MonitorMessageItem::HMI),
      websocket_(websocket),
      map_service_(map_service),
      hmi_ws_(hmi_websocket) {
  if (websocket_) {
    RegisterMessageHandlers();
  }
}

void HMI::StartStream(const double& time_interval_ms,
                      const std::string& channel_name,
                      nlohmann::json* subscribe_param) {
  AINFO << "Start HMIStatus updater timer with data sending frequency: "
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

void HMI::Start(DvCallback callback_api) { hmi_worker_->Start(callback_api); }

void HMI::Stop() { hmi_worker_->Stop(); }

void HMI::StopStream(const std::string& channel_name) {
  AINFO << "HMIStatus updater timer has been stopped";
  if (timer_) {
    timer_->Stop();
  }
}

void HMI::RegisterMessageHandlers() {
  // Broadcast HMIStatus to clients when status changed.
  hmi_worker_->RegisterStatusUpdateHandler(
      [this](const bool status_changed, HMIStatus* status) {
        if (!status_changed) {
          // Status doesn't change, skip broadcasting.
          return;
        }
        // websocket_->BroadcastData(
        //     JsonUtil::ProtoToTypedJson("HMIStatus", *status).dump());
        if (status->current_map().empty()) {
          monitor_log_buffer_.WARN("You haven't selected a map yet!");
        }
        if (status->current_vehicle().empty()) {
          monitor_log_buffer_.WARN("You haven't selected a vehicle yet!");
        }
      });

  // Send current status and vehicle param to newly joined client.
  // websocket_->RegisterConnectionReadyHandler(
  //     [this](WebSocketHandler::Connection* conn) {
  //       // SendStatus(conn);
  //       SendVehicleParam(conn);
  //     });

  websocket_->RegisterMessageHandler(
      "HMIAction",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        // Run HMIWorker::Trigger(action) if json is {action: "<action>"}
        // Run HMIWorker::Trigger(action, value) if "value" field is provided.

        // response is used for interfaces that require a reply
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to start hmi action: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;

        std::string action;
        if (!JsonUtil::GetStringByPath(json, "data.info.action", &action)) {
          AERROR << "Truncated HMIAction request.";
          return;
        }
        HMIAction hmi_action;
        if (!HMIAction_Parse(action, &hmi_action)) {
          AERROR << "Invalid HMIAction string: " << action;
          return;
        }
        bool is_ok = false;
        std::string value;
        if (JsonUtil::GetStringByPath(json, "data.info.value", &value)) {
          is_ok = hmi_worker_->Trigger(hmi_action, value);
        } else {
          is_ok = hmi_worker_->Trigger(hmi_action);
        }

        // Extra works for current Dreamview.
        if (hmi_action == HMIAction::CHANGE_VEHICLE) {
          // Reload lidar params for point cloud service.
          PointCloudUpdater::LoadLidarHeight(FLAGS_lidar_height_yaml);
          SendVehicleParam();
        } else if (hmi_action == HMIAction::CHANGE_MAP) {
          response["data"]["info"]["data"]["isOk"] = is_ok;
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] =
              is_ok ? "Success" : "Failed to change map";
          websocket_->SendData(conn, response.dump());
        } else if (hmi_action == HMIAction::CHANGE_OPERATION ||
                  hmi_action == HMIAction::CHANGE_MODE) {
          response["data"]["info"]["data"]["isOk"] = true;
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = "Success";
          websocket_->SendData(conn, response.dump());
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

  // websocket_->RegisterMessageHandler(
  //     "HMIStatus",
  //     [this](const Json& json, WebSocketHandler::Connection* conn) {
  //       SendStatus(conn);
  //     });

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

  websocket_->RegisterMessageHandler(
      "StartPlayRecorder",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        std::string request_id;
        Json response({});
        response["action"] = "response";
        Json info({});
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          // 失败情况
          info["code"] = -1;
          info["message"] = "Miss requestId";
          AERROR << "Miss required field requestId to execute play recorder "
                    "action.";

        } else {
          bool exec_action_result = hmi_worker_->RePlayRecord();
          info["code"] = exec_action_result ? 0 : -1;
          if (!exec_action_result) {
            info["message"] = "Failed to start play recorder";
          }
        }
        response["data"]["info"] = info;
        if (!request_id.empty()) {
          response["data"]["requestId"] = request_id;
        }
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "ResetRecordProgress",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        std::string request_id;
        std::string progress_str;
        Json response({});
        response["action"] = "response";
        Json info({});
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          info["code"] = -1;
          info["message"] = "Miss requestId";
          AERROR << "Miss required field requestId to execute play recorder "
                    "action.";

        } else if (!JsonUtil::GetStringByPath(json, "data.info.progress",
                                              &progress_str)) {
          info["code"] = -1;
          info["message"] = "Miss progress";
          AERROR << "Miss required field progress to reset record progress.";
        } else {
          double progress;
          bool valid_progress = true;
          try {
            progress = std::stod(progress_str);
            AINFO << "Progress: " << progress;
          } catch (const std::invalid_argument& ia) {
            AERROR << "Invalid argument progress:  " << progress_str;
            valid_progress = false;
          } catch (const std::out_of_range& e) {
            AERROR << "Argument is out of range:  " << progress_str;
            valid_progress = false;
          }
          if (valid_progress) {
            bool result = hmi_worker_->ResetRecordProgress(progress);
            info["code"] = result ? 0 : -1;
            if (!result) {
              info["message"] = "Failed to start play recorder";
            }
          } else {
            info["code"] = -1;
            info["message"] = "Failed to get proress value";
          }
        }
        response["data"]["info"] = info;
        if (!request_id.empty()) {
          response["data"]["requestId"] = request_id;
        }
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "PlayRecorderAction",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        std::string request_id;
        std::string action_type;
        double progress;
        Json response({});
        response["action"] = "response";
        Json info({});
        if (!JsonUtil::GetStringByPath(json, "data.info.actionType",
                                       &action_type)) {
          info["code"] = -1;
          info["message"] = "Error param";
          AERROR << "Failed to get param to execute play recorder action.";
        } else if (!JsonUtil::GetStringByPath(json, "data.requestId",
                                              &request_id)) {
          // 失败情况
          info["code"] = -1;
          info["message"] = "Miss requestId";
          AERROR << "Miss required field requestId to execute play recorder "
                    "action.";

        } else if (action_type.compare("continue") == 0 &&
                   !JsonUtil::GetNumberByPath(json, "data.info.progress",
                                              &progress)) {
          AERROR << "Miss required field progress to continue play record!";
          info["code"] = -1;
          info["message"] = "Miss progress";
        } else {
          bool exec_action_result =
              hmi_worker_->handlePlayRecordProcess(action_type);
          info["code"] = exec_action_result ? 0 : -1;
          if (!exec_action_result) {
            info["message"] =
                "Failed to execute play recorder action: " + action_type;
          }
        }
        response["data"]["info"] = info;
        if (!request_id.empty()) {
          response["data"]["requestId"] = request_id;
        }
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "StartDataRecorder",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to start data recorder: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        bool ret = hmi_worker_->StartDataRecorder();
        if (ret) {
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = "Success";
        } else {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "Failed to start data recorder:Failed to start the "
              "cyber_recorder process";
        }
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "StopDataRecorder",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to start data recorder: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        bool ret = hmi_worker_->StopDataRecorder();
        if (ret) {
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = "Success";
        } else {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "Failed to stop data recorder: failed to stop the cyber_recorder "
              "process";
        }
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "SaveDataRecorder",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        Json response;
        response["action"] = "response";
        std::string request_id, new_name;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to save record: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        } else if (!JsonUtil::GetStringByPath(json, "data.info.newName",
                                              &new_name)) {
          AERROR << "Failed to save record: newName not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss newName";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        bool ret = hmi_worker_->SaveDataRecorder(new_name);
        if (ret) {
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = "Success";
        } else {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "Failed to save record: a file with the same name exists";
        }
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "DeleteDataRecorder",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to start data recorder: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        bool ret = hmi_worker_->DeleteDataRecorder();
        if (ret) {
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = "Success";
        } else {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Failed to delete the record";
        }
        websocket_->SendData(conn, response.dump());
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

void HMI::OnTimer(const std::string& channel_name) { PublishMessage(); }

void HMI::PublishMessage(const std::string& channel_name) {
  StreamData response;
  response.set_action("stream");
  response.set_data_name("hmistatus");
  std::string hmi_status_str;
  hmi_worker_->GetStatus().SerializeToString(&hmi_status_str);
  std::vector<uint8_t> byte_data(hmi_status_str.begin(), hmi_status_str.end());
  response.set_data(&(byte_data[0]), byte_data.size());
  response.set_type("hmistatus");
  std::string response_str;
  response.SerializeToString(&response_str);
  hmi_ws_->BroadcastBinaryData(response_str);
}

bool HMI::UpdateScenarioSetToStatus(const std::string& scenario_set_id,
                                    const std::string& scenario_set_name) {
  return hmi_worker_->UpdateScenarioSetToStatus(scenario_set_id,
                                                scenario_set_name);
}

bool HMI::UpdateDynamicModelToStatus(const std::string& dynamic_model_name) {
  return hmi_worker_->UpdateDynamicModelToStatus(dynamic_model_name);
}

bool HMI::UpdateMapToStatus(const std::string& map_name) {
  return hmi_worker_->UpdateMapToStatus(map_name);
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

void HMI::GetCurrentScenarioEndPoint(double& x, double& y) {
  hmi_worker_->GetCurrentScenarioEndPoint(x, y);
}

bool HMI::StartSimObstacle() { return hmi_worker_->StartSimObstacle(); }

bool HMI::StopSimObstacle() { return hmi_worker_->StopSimObstacle(); }

}  // namespace dreamview
}  // namespace apollo
