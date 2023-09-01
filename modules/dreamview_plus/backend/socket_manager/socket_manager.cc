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

#include "modules/dreamview_plus/backend/socket_manager/socket_manager.h"

#include <vector>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/message/protobuf_factory.h"
#include "modules/common/util/json_util.h"
#include "modules/dreamview_plus/backend/common/dreamview_gflags.h"
#include "modules/dreamview_plus/backend/updater/updater_with_channels_base.h"

namespace apollo {
namespace dreamview {

using apollo::common::util::JsonUtil;
using Json = nlohmann::json;
using cyber::common::GetProtoFromFile;

SocketManager::SocketManager(WebSocketHandler *websocket,
                             UpdaterManager *updater_manager)
    : enabled_(false),
      websocket_(websocket),
      updater_manager_(updater_manager) {
  RegisterDataHandlers();
  RegisterMessageHandlers();
}

void SocketManager::RegisterDataHandlers() {
  enabled_ = GetProtoFromFile(FLAGS_data_handler_config_path,
                              &data_handler_base_conf_);
  if (!enabled_) {
    AERROR << "Unable to parse data handler configuration from file "
           << FLAGS_data_handler_config_path;
  }

  // 遍历 然后register
  // for (const auto& data_handler_iter :
  // data_handler_base_conf_.data_handler_info()) {
  //   const std::string& data_name = data_handler_iter.first;
  //   const DataHandlerInfo& data_handler_info = data_handler_iter.second;
  //   RegisterDataHandler(data_handler_info);
  // }
}

void SocketManager::RegisterMessageHandlers() {
  websocket_->RegisterConnectionReadyHandler(
      [this](WebSocketHandler::Connection *conn) { BrocastDataHandlerConf(); });

  websocket_->RegisterMessageHandler(
      "subscribe",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["action"] = "response";
        response["data"]["info"] = {};
        response["data"]["info"]["code"] = this->Subscribe(json) ? 0 : -1;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "unsubscribe",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["action"] = "response";
        response["data"]["info"] = {};
        response["data"]["info"]["code"] = this->UnSubscribe(json) ? 0 : -1;
        websocket_->SendData(conn, response.dump());
      });
}

bool SocketManager::GetDataUpdaterChannels(const std::string &updater_path,
                                           std::vector<std::string> *channels) {
  UpdaterBase *updater = updater_manager_->GetUpdater(updater_path);
  UpdaterWithChannelsBase *updater_with_channels =
      dynamic_cast<UpdaterWithChannelsBase *>(updater);
  if (updater_with_channels == nullptr) {
    return false;
  }
  updater_with_channels->GetChannelMsg(channels);
  return true;
}

bool SocketManager::Subscribe(const Json &json) {
  const std::string url = json["data"]["info"]["websocketName"];
  double time_interval_ms = json["data"]["info"]["dataFrequencyMs"];
  std::string channel_name;
  if (!JsonUtil::GetStringByPath(json, "data.info.channelName",
                                 &channel_name)) {
    channel_name = "";
  }
  return updater_manager_->Start(url, time_interval_ms, channel_name);
}

bool SocketManager::UnSubscribe(const Json &json) {
  const std::string url = json["data"]["info"]["websocketName"];
  std::string channel_name;
  if (!JsonUtil::GetStringByPath(json, "data.info.channelName",
                                 &channel_name)) {
    channel_name = "";
  }
  return updater_manager_->Stop(url, channel_name);
}

Json SocketManager::ClearDataHandlerChannelMsgs() {
  Json response_data({});
  Json data({});

  if (enabled_) {
    for (auto iter = data_handler_conf_.mutable_data_handler_info()->begin();
         iter != data_handler_conf_.mutable_data_handler_info()->end();
         iter++) {
      if (iter->second.different_for_channels()) {
        const std::string &data_type = (*iter).first;
        (*data_handler_conf_.mutable_data_handler_info())[data_type]
            .clear_channels();
      }
    }
  }
  data = JsonUtil::ProtoToTypedJson("data", data_handler_conf_);
  data.erase("type");
  response_data["info"] = data;
  response_data["info"]["code"] = 0;
  return response_data;
}

Json SocketManager::GetDataHandlerInfo() {
  Json response_data({});
  Json data({});
  data_handler_conf_.Clear();

  if (enabled_) {
    for (auto iter =
             data_handler_base_conf_.mutable_data_handler_info()->begin();
         iter != data_handler_base_conf_.mutable_data_handler_info()->end();
         iter++) {
      const std::string &data_type = (*iter).first;
      const std::string &updater_path = (*iter).second.data_name();
      DataHandlerInfo updater_info;
      updater_info.CopyFrom(iter->second);
      std::string proto_desc;
      apollo::cyber::message::ProtobufFactory::Instance()->GetDescriptorString(
          data_type, &proto_desc);
      if (proto_desc.empty()) {
        AWARN << "Cannot find proto descriptor for message type " << data_type;
        continue;
      }
      updater_info.set_proto_desc(proto_desc);
      if (iter->second.different_for_channels()) {
        std::vector<std::string> channels;
        if (!GetDataUpdaterChannels(updater_path, &channels)) {
          continue;
        }
        for (auto iter = channels.begin(); iter != channels.end(); iter++) {
          updater_info.add_channels(*iter);
        }
      }
      (*data_handler_conf_.mutable_data_handler_info())[data_type] =
          updater_info;
    }
  }
  data = JsonUtil::ProtoToTypedJson("data", data_handler_conf_);
  data.erase("type");
  response_data["info"] = data;
  response_data["info"]["code"] = 0;
  return response_data;
}

void SocketManager::BrocastDataHandlerConf(bool clear_channel_msg) {
  Json response({});
  response["data"] =
      clear_channel_msg ? ClearDataHandlerChannelMsgs() : GetDataHandlerInfo();
  response["action"] = "metadata";
  websocket_->BroadcastData(response.dump());
}
}  // namespace dreamview
}  // namespace apollo
