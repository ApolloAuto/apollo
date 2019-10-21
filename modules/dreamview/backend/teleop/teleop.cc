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

namespace apollo {
namespace dreamview {

using Json = nlohmann::json;
using ::google::protobuf::util::MessageToJsonString;
using modules::car1::network::ModemInfo;
using modules::car1::teleop::DaemonServiceCmd;

TeleopService::TeleopService(WebSocketHandler *websocket)
    : node_(cyber::CreateNode("teleop")), websocket_(websocket) {
  RegisterMessageHandlers();

  teleop_status_["audio"] = false;
  teleop_status_["mic"] = false;
  teleop_status_["video"] = false;
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
      [this](WebSocketHandler::Connection *conn) { SendStatus(conn); });
  // Start/Stop local and remote audio
  websocket_->RegisterMessageHandler(
      "ToggleAudio",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // TODO
        {
          boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
          teleop_status_["audio"] = !teleop_status_["audio"];

          // turn on/off the mic based on the audio status
          teleop_status_["mic"] = teleop_status_["audio"];
        }
      });
  // Mute/Unmute local microphone
  websocket_->RegisterMessageHandler(
      "ToggleMic",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // TODO
        {
          boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
          teleop_status_["mic"] = !teleop_status_["mic"];
        }
      });
  // Start/stop local decoder and viewer, start/stop remote encoder and
  // compositor
  websocket_->RegisterMessageHandler(
      "ToggleVideo",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // TODO
        {
          boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
          teleop_status_["video"] = !teleop_status_["video"];
        }
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
      "RequestTeleopStatus",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        SendStatus(conn);
      });
}

void TeleopService::UpdateModemInfo(
    const std::shared_ptr<ModemInfo> &modem_info) {
  // TODO simplify data and only send necessary info for display
  if (modem_info->has_provider() && modem_info->has_technology()) {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    teleop_status_["modems"][modem_info->provider()] = modem_info->technology();
  }
}

void TeleopService::SendStatus(WebSocketHandler::Connection *conn) {
  std::string to_send;
  {
    boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
    to_send = teleop_status_.dump();
  }
  websocket_->SendData(conn, to_send);
}

}  // namespace dreamview
}  // namespace apollo
