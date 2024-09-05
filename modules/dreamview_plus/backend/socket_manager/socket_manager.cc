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
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview_plus/backend/record_player/record_player_factory.h"
#include "modules/dreamview_plus/backend/updater/updater_with_channels_base.h"
namespace apollo {
namespace dreamview {

using apollo::common::util::JsonUtil;
using Json = nlohmann::json;
using cyber::common::GetProtoFromFile;

std::map<std::string, std::pair<std::string, std::string>> filter_info = {
    {"apollo.dreamview.Obstacles", {"perception.PerceptionObstacles", ""}},
    {"apollo.dreamview.CameraUpdate", {"drivers.Image", ""}},
    {"apollo.dreamview.PointCloud", {"PointCloud", "sensor"}}};

SocketManager::SocketManager(WebSocketHandler *websocket,
                             UpdaterManager *updater_manager,
                             DvPluginManager *dv_plugin_manager)
    : enabled_(false),
      websocket_(websocket),
      updater_manager_(updater_manager),
      dv_plugin_manager_(dv_plugin_manager) {
  RegisterDataHandlers();
  RegisterMessageHandlers();
  auto channel_manager =
      apollo::cyber::service_discovery::TopologyManager::Instance()
          ->channel_manager();
  auto topology_callback =
      [this](const apollo::cyber::proto::ChangeMsg &change_msg) {
        this->RefreshChannels(change_msg);
      };
  channel_manager->AddChangeListener(topology_callback);
}

void SocketManager::RegisterDataHandlers() {
  enabled_ = GetProtoFromFile(FLAGS_data_handler_config_path,
                              &data_handler_base_conf_);
  if (!enabled_) {
    AERROR << "Unable to parse data handler configuration from file "
           << FLAGS_data_handler_config_path;
  }
  for (auto &data_handler_iter :
       dv_plugin_manager_->data_handler_conf_.data_handler_info()) {
    if (data_handler_base_conf_.data_handler_info().find(
            data_handler_iter.first) !=
        data_handler_base_conf_.data_handler_info().end()) {
      AERROR << "There are duplicate updater handlers between dv and plugins";
      continue;
    }
    auto data_handler_info =
        data_handler_base_conf_.mutable_data_handler_info();
    (*data_handler_info)[data_handler_iter.first] = data_handler_iter.second;
  }
  AINFO << "data_handler_base_conf_: " << data_handler_base_conf_.DebugString();

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

bool SocketManager::ModifyUpdaterChannels(const std::string &updater_path,
                                          const std::string &channel_name,
                                          const std::string &operation) {
  UpdaterBase *updater = updater_manager_->GetUpdater(updater_path);
  UpdaterWithChannelsBase *updater_with_channels =
      dynamic_cast<UpdaterWithChannelsBase *>(updater);
  if (updater_with_channels == nullptr) {
    return false;
  }
  auto iter = std::find(updater_with_channels->channels_.begin(),
                        updater_with_channels->channels_.end(), channel_name);
  if (operation == "join" && iter == updater_with_channels->channels_.end()) {
    updater_with_channels->channels_.emplace_back(channel_name);
  } else if (operation == "leave" &&
             iter != updater_with_channels->channels_.end()) {
    updater_with_channels->channels_.erase(iter);
  }
  return true;
}

bool SocketManager::Subscribe(const Json &json) {
  const std::string url = json["data"]["info"]["websocketName"];
  double time_interval_ms = 0;
  JsonUtil::GetNumberByPath(json, "data.info.dataFrequencyMs",
                            &time_interval_ms);
  std::string channel_name;
  if (!JsonUtil::GetStringByPath(json, "data.info.channelName",
                                 &channel_name)) {
    channel_name = "";
  }
  Json subscribe_param = {};
  std::vector<std::string> json_path = {"data", "info", "param"};
  JsonUtil::GetJsonByPath(json, json_path, &subscribe_param);
  return updater_manager_->Start(url, time_interval_ms, channel_name,
                                 &subscribe_param);
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
    complete_channel_count_.clear();
    data_handler_channel_count_.clear();
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
  data_handler_channel_count_.clear();
  complete_channel_count_.clear();

  if (enabled_) {
    for (auto iter =
             data_handler_base_conf_.mutable_data_handler_info()->begin();
         iter != data_handler_base_conf_.mutable_data_handler_info()->end();
         iter++) {
      const std::string &data_type = (*iter).first;
      DataHandlerInfo updater_info;
      updater_info.CopyFrom(iter->second);
      if (data_type == FLAGS_cyber_channels_key) {
        // For cyber channels
        auto topology_manager =
            apollo::cyber::service_discovery::TopologyManager::Instance();
        auto channel_manager = topology_manager->channel_manager();

        std::vector<apollo::cyber::proto::RoleAttributes> role_attributes;
        channel_manager->GetWriters(&role_attributes);

        for (const auto &role : role_attributes) {
          if (!complete_channel_count_[role.channel_name()]) {
            std::string proto_path;
            apollo::cyber::message::ProtobufFactory::Instance()->GetProtoPath(
                role.message_type(), proto_path);
            if (proto_path.empty()) {
              AWARN << "Cannot find proto location for message type "
                    << role.message_type();
              continue;
            }
            auto channel_item = updater_info.add_channels();
            channel_item->set_proto_path(proto_path);
            channel_item->set_channel_name(role.channel_name());
            channel_item->set_msg_type(role.message_type());
          }
          complete_channel_count_[role.channel_name()]++;
        }
      } else {
        // For other channels
        const std::string &updater_path = (*iter).second.data_name();
        std::string proto_path;
        apollo::cyber::message::ProtobufFactory::Instance()->GetProtoPath(
            data_type, proto_path);
        if (proto_path.empty()) {
          AWARN << "Cannot find proto location for message type " << data_type;
          continue;
        }
        updater_info.set_proto_path(proto_path);
        if (iter->second.different_for_channels()) {
          std::vector<std::string> channels;
          if (!GetDataUpdaterChannels(updater_path, &channels)) {
            continue;
          }
          for (auto iter = channels.begin(); iter != channels.end(); iter++) {
            if (!data_handler_channel_count_[*iter]) {
              auto channel_item = updater_info.add_channels();
              channel_item->set_channel_name(*iter);
            }
            data_handler_channel_count_[*iter]++;
          }
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

bool SocketManager::UpdateUpdaterInfo(
    const apollo::cyber::proto::ChangeMsg &change_msg,
    DataHandlerInfo &updater_info) {
  auto role_attr = change_msg.role_attr();
  auto channel_item = updater_info.add_channels();
  channel_item->set_channel_name(role_attr.channel_name());
  if (::apollo::cyber::proto::OperateType::OPT_JOIN ==
      change_msg.operate_type()) {
    channel_item->set_msg_type(role_attr.message_type());
    std::string proto_path;
    apollo::cyber::message::ProtobufFactory::Instance()->GetProtoPath(
        role_attr.message_type(), proto_path);
    if (proto_path.empty()) {
      AWARN << "Cannot find proto location for message type "
            << role_attr.message_type();
      return false;
    }
    channel_item->set_proto_path(proto_path);
  }
  return true;
}

bool SocketManager::UpdateCyberChannels(
    const apollo::cyber::proto::ChangeMsg &change_msg,
    DataHandlerInfo &updater_info) {
  std::string channel_name = change_msg.role_attr().channel_name();

  if (::apollo::cyber::proto::OperateType::OPT_JOIN ==
      change_msg.operate_type()) {
    // Used to extract the channel of the complete set.
    if (!complete_channel_count_[channel_name]) {
      if (!UpdateUpdaterInfo(change_msg, updater_info)) return false;
      complete_channel_count_[channel_name]++;
      return true;
    }
    complete_channel_count_[channel_name]++;
  } else {
    complete_channel_count_[channel_name]--;
    if (complete_channel_count_[channel_name] == 0) {
      if (!UpdateUpdaterInfo(change_msg, updater_info)) return false;
      return true;
    }
  }
  return false;
}

void SocketManager::RefreshDataHandlerChannels(
    const apollo::cyber::proto::ChangeMsg &change_msg,
    DataHandlerConf &data_handler_conf_diff, bool &flag) {
  auto role_attr = change_msg.role_attr();
  std::string message_type = role_attr.message_type();
  std::string channel_name = role_attr.channel_name();

  // Used to filter the channel of the current data packet and the channel in
  // non-broadcast packet mode.
  std::vector<std::string> records;
  std::vector<std::string> other_record_node_name;
  auto *record_player_factory = RecordPlayerFactory::Instance();
  record_player_factory->GetAllRecords(&records);
  const std::string current_record = record_player_factory->GetCurrentRecord();
  for (auto iter = records.begin(); iter != records.end(); iter++) {
    if (current_record.empty() || *iter != current_record) {
      std::string other_node_name = "record_player_factory_" + *iter;
      other_record_node_name.push_back(other_node_name);
    }
  }

  std::string node_name = role_attr.node_name();
  for (auto iter = data_handler_base_conf_.mutable_data_handler_info()->begin();
       iter != data_handler_base_conf_.mutable_data_handler_info()->end();
       iter++) {
    const std::string &data_type = (*iter).first;
    DataHandlerInfo updater_info;
    updater_info.set_data_name(iter->second.data_name());
    updater_info.set_different_for_channels(
        iter->second.different_for_channels());

    if (data_type == FLAGS_cyber_channels_key) {
      // For cyber channels
      flag = UpdateCyberChannels(change_msg, updater_info);
    } else {
      // For other channels
      if (!iter->second.different_for_channels()) {
        (*data_handler_conf_diff.mutable_data_handler_info())[data_type] =
            updater_info;
        continue;
      }

      int index = 0;
      if (filter_info.find(data_type) != filter_info.end() &&
          !filter_info[data_type].first.empty()) {
        index = message_type.rfind(filter_info[data_type].first);
      }
      int index_channel = 0;
      if (filter_info.find(data_type) != filter_info.end() &&
          !filter_info[data_type].second.empty()) {
        index_channel = channel_name.rfind(filter_info[data_type].second);
      }

      if (index != -1 && index_channel != -1 &&
          (current_record.empty() ||
           std::find(other_record_node_name.begin(),
                     other_record_node_name.end(),
                     node_name) == other_record_node_name.end())) {
        if (::apollo::cyber::proto::OperateType::OPT_JOIN ==
            change_msg.operate_type()) {
          if (!data_handler_channel_count_[channel_name]) {
            flag = true;
            auto channel_item = updater_info.add_channels();
            channel_item->set_channel_name(channel_name);
            if (!ModifyUpdaterChannels(iter->second.data_name(), channel_name,
                                       "join"))
              continue;
          }
          data_handler_channel_count_[channel_name]++;
        } else {
          data_handler_channel_count_[channel_name]--;
          if (data_handler_channel_count_[channel_name] == 0) {
            flag = true;
            auto channel_item = updater_info.add_channels();
            channel_item->set_channel_name(channel_name);
            if (!ModifyUpdaterChannels(iter->second.data_name(), channel_name,
                                       "leave"))
              continue;
          }
        }
      }
    }
    (*data_handler_conf_diff.mutable_data_handler_info())[data_type] =
        updater_info;
  }
}

void SocketManager::RefreshChannels(
    const apollo::cyber::proto::ChangeMsg &change_msg) {
  // Just look at the writer's
  if (::apollo::cyber::proto::RoleType::ROLE_READER == change_msg.role_type()) {
    return;
  }

  DataHandlerConf data_handler_conf_diff;
  bool flag = false;

  RefreshDataHandlerChannels(change_msg, data_handler_conf_diff, flag);

  // If the channel is not used.
  if (flag == false) {
    return;
  }

  Json response({});
  Json data({});
  data = JsonUtil::ProtoToTypedJson("data", data_handler_conf_diff);
  data.erase("type");
  response["data"]["info"] = data;
  response["data"]["info"]["code"] = 0;
  response["action"] =
      ::apollo::cyber::proto::OperateType::OPT_JOIN == change_msg.operate_type()
          ? "join"
          : "leave";
  websocket_->BroadcastData(response.dump());
}

}  // namespace dreamview
}  // namespace apollo
