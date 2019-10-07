/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/dreamview/backend/teleop/teleop.h"

#include "cyber/common/log.h"
#include "google/protobuf/util/json_util.h"
#include "third_party/json/json.hpp"

namespace apollo {
namespace dreamview {

using Json = nlohmann::json;
using ::google::protobuf::util::MessageToJsonString;
using modules::car1::network::ModemInfo;
using modules::car1::teleop::DaemonServiceCmd;

TeleopService::TeleopService(WebSocketHandler *websocket)
    : node_(cyber::CreateNode("teleop")), websocket_(websocket) {
  RegisterMessageHandlers();
}

void TeleopService::Start() {
  // TODO get topic names from proto
  // TODO update proto to get all modems' info combined with rank
  modem_info_reader_ = node_->CreateReader<ModemInfo>(
      "/apollo/networks/modems",
      [this](const std::shared_ptr<ModemInfo> &msg) { UpdateModemInfo(msg); });
  daemon_cmd_writer_ = node_->CreateWriter<DaemonServiceCmd>(
      "/apollo/teleop/daemon_service/cmd");
}

void TeleopService::RegisterMessageHandlers() {
  // Send current teleop status to the new client.
  websocket_->RegisterConnectionReadyHandler(
      [this](WebSocketHandler::Connection *conn) {
        Json response;
        response["audio"] = audio_enabled_;
        response["mic"] = mic_enabled_;
        response["video"] = video_enabled_;
        websocket_->SendData(conn, response.dump());
      });
  // Start/Stop local and remote audio
  websocket_->RegisterMessageHandler(
      "ToggleAudio",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // TODO
        audio_enabled_ = !audio_enabled_;
      });
  // Mute/Unmute local microphone
  websocket_->RegisterMessageHandler(
      "ToggleMic",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // TODO
        mic_enabled_ = !mic_enabled_;
      });
  // Start/stop local decoder and viewer, start/stop remote encoder and
  // compositor
  websocket_->RegisterMessageHandler(
      "ToggleVideo",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // TODO
        video_enabled_ = !video_enabled_;
      });
  // Issue pull-over command to remote
  websocket_->RegisterMessageHandler(
      "PullOver", [this](const Json &json, WebSocketHandler::Connection *conn) {
        // TODO
      });
  // Issue emergency-stop command to remote
  websocket_->RegisterMessageHandler(
      "EStop", [this](const Json &json, WebSocketHandler::Connection *conn) {
        // TODO
      });
  // Request to get updated modem info for client display
  websocket_->RegisterMessageHandler(
      "GetModemInfo",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        std::string to_send;
        {
          boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
          MessageToJsonString(modem_info_, &to_send);
        }
        websocket_->SendData(conn, to_send);
      });
}

void TeleopService::UpdateModemInfo(
    const std::shared_ptr<ModemInfo> &modem_info) {
  {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    // TODO simplify data and only send necessary info for display
    // update modem_info_
  }
}

}  // namespace dreamview
}  // namespace apollo
