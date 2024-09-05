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
#include <utility>

#include "google/protobuf/util/json_util.h"

#include "modules/dreamview/proto/preprocess_table.pb.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/json_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview_plus/backend/point_cloud/point_cloud_updater.h"

namespace apollo {
namespace dreamview {

using apollo::common::util::JsonUtil;
using apollo::cyber::common::SetProtoToASCIIFile;
using google::protobuf::util::JsonStringToMessage;
using Json = WebSocketHandler::Json;

HMI::HMI(WebSocketHandler* websocket, MapService* map_service,
         WebSocketHandler* hmi_websocket)
    : monitor_log_buffer_(apollo::common::monitor::MonitorMessageItem::HMI),
      hmi_worker_(new HMIWorker(monitor_log_buffer_)),
      websocket_(websocket),
      map_service_(map_service),
      hmi_ws_(hmi_websocket) {
  if (websocket_) {
    RegisterDBMessageHandlers();
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

void HMI::RegisterDBMessageHandlers() {
  websocket_->RegisterMessageHandler(
      "AddOrModifyObjectToDB",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR
              << "Failed to add or modify object to DB: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        std::string key, value;
        if (!JsonUtil::GetStringByPath(json, "data.info.key", &key)) {
          AERROR << "Failed to add or modify object to DB: key not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss key";
          websocket_->SendData(conn, response.dump());
          return;
        } else if (!JsonUtil::GetStringByPath(json, "data.info.value",
                                              &value)) {
          AERROR << "Failed to add or modify object to DB: value not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss value";
          websocket_->SendData(conn, response.dump());
          return;
        }
        bool ret = hmi_worker_->AddOrModifyObjectToDB(key, value);
        if (ret) {
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = "Success";
        } else {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "Failed to add or modify object to DB";
        }
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "GetObjectFromDB",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to get object from DB: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        std::string key, value;
        if (!JsonUtil::GetStringByPath(json, "data.info.key", &key)) {
          AERROR << "Failed to get object from DB: key not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss key";
          websocket_->SendData(conn, response.dump());
          return;
        }
        std::string ret = hmi_worker_->GetObjectFromDB(key);
        response["data"]["info"]["data"] = ret;
        response["data"]["info"]["code"] = 0;
        response["data"]["info"]["message"] = "Success";
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "DeleteObjectToDB",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to delete object to DB: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        std::string key, value;
        if (!JsonUtil::GetStringByPath(json, "data.info.key", &key)) {
          AERROR << "Failed to delete object to DB: key not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss key";
          websocket_->SendData(conn, response.dump());
          return;
        }
        bool ret = hmi_worker_->DeleteObjectToDB(key);
        if (ret) {
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = "Success";
        } else {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Failed to delete object to DB";
        }
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "GetTuplesWithTypeFromDB",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR
              << "Failed to get tuples with type from DB: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        std::string type;
        if (!JsonUtil::GetStringByPath(json, "data.info.type", &type)) {
          AERROR << "Failed to get tuples with type from DB: type not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss type";
          websocket_->SendData(conn, response.dump());
          return;
        }
        auto ret = hmi_worker_->GetTuplesWithTypeFromDB(type);
        Json tuples = Json::array();
        for (const auto& item : ret) {
          Json tuple;
          tuple["key"] = item.first;
          tuple["value"] = item.second;
          tuples.push_back(tuple);
        }
        response["data"]["info"]["data"] = tuples;
        response["data"]["info"]["code"] = 0;
        response["data"]["info"]["message"] = "Success";
        websocket_->SendData(conn, response.dump());
      });
}

void HMI::RegisterMessageHandlers() {
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
        bool ret;
        if (hmi_worker_->GetStatus().current_operation() ==
            HMIModeOperation::Waypoint_Follow) {
          ret = hmi_worker_->StartRtkDataRecorder();
        } else {
          ret = hmi_worker_->StartDataRecorder();
        }
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
        bool ret;
        if (hmi_worker_->GetStatus().current_operation() ==
            HMIModeOperation::Waypoint_Follow) {
          ret = hmi_worker_->StopRtkDataRecorder();
        } else {
          ret = hmi_worker_->StopDataRecorder();
        }
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
      "StartPlayRtkRecorder",
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
        if (hmi_worker_->GetStatus().current_operation() !=
            HMIModeOperation::Waypoint_Follow) {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "The current operation is not a Waypoint_ Follow";
        }
        Json result = hmi_worker_->StartPlayRtkRecorder();
        response["data"]["info"]["code"] = result["isOk"] ? 0 : -1;
        response["data"]["info"]["message"] =
            result["isOk"] ? "Success" : result["error"];
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "StopPlayRtkRecorder",
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
        if (hmi_worker_->GetStatus().current_operation() !=
            HMIModeOperation::Waypoint_Follow) {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "The current operation is not a Waypoint_ Follow";
        }
        bool ret = hmi_worker_->StopPlayRtkRecorder();
        if (ret) {
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = "Success";
        } else {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "Failed to stop play data recorder process";
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
        int ret;
        if (hmi_worker_->GetStatus().current_operation() ==
            HMIModeOperation::Waypoint_Follow) {
          ret = hmi_worker_->SaveRtkDataRecorder(new_name);
        } else {
          ret = hmi_worker_->SaveDataRecorder(new_name);
        }
        if (ret == 1) {
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = "Success";
        } else if (ret == -1) {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "Failed to save record: a file with the same name exists";
        } else if (ret == -2) {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "Failed to save the record: the dreamview recording record does "
              "not exist, please record through dreamview";
        } else {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] =
              "Failed to save record: please try again or check whether your "
              "record is legal";
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
        bool ret;
        if (hmi_worker_->GetStatus().current_operation() ==
            HMIModeOperation::Waypoint_Follow) {
          ret = hmi_worker_->DeleteRtkDataRecorder();
        } else {
          ret = hmi_worker_->DeleteDataRecorder();
        }
        if (ret) {
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = "Success";
        } else {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Failed to delete the record";
        }
        websocket_->SendData(conn, response.dump());
      });

  // To speed up the preloading process, fetching from hmi is relatively slow.
  websocket_->RegisterMessageHandler(
      "GetInitData",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to get current mode: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        std::string current_mode = hmi_worker_->GetStatus().current_mode();
        auto current_operation = hmi_worker_->GetStatus().current_operation();
        response["data"]["info"]["data"]["currentMode"] = current_mode;
        response["data"]["info"]["data"]["currentOperation"] =
            HMIModeOperation_Name(current_operation);
        response["data"]["info"]["code"] = 0;
        response["data"]["info"]["message"] = "Success";
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "StartTerminal",
      [this](const Json& json, WebSocketHandler::Connection* conn) {
        Json response;
        response["action"] = "response";
        std::string request_id;
        if (!JsonUtil::GetStringByPath(json, "data.requestId", &request_id)) {
          AERROR << "Failed to start terminal: requestId not found.";
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Miss requestId";
          websocket_->SendData(conn, response.dump());
          return;
        }
        response["data"]["requestId"] = request_id;
        bool ret = hmi_worker_->StartTerminal();
        if (ret) {
          response["data"]["info"]["code"] = 0;
          response["data"]["info"]["message"] = "Success";
        } else {
          response["data"]["info"]["code"] = -1;
          response["data"]["info"]["message"] = "Failed to start terminal";
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

bool HMI::isProcessRunning(const std::string& process_name) {
  return hmi_worker_->isProcessRunning(process_name);
}

}  // namespace dreamview
}  // namespace apollo
