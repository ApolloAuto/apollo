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

/**
 * @file socket_manager.h
 * @brief SocketManager to manage all websocket
 * @author LiJin
 * @date 2023/06/20
 */

#pragma once

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"

#include "modules/dreamview_plus/proto/data_handler.pb.h"

#include "cyber/common/log.h"
#include "cyber/service_discovery/topology_manager.h"
#include "modules/dreamview/backend/common/handlers/websocket_handler.h"
#include "modules/dreamview_plus/backend/dv_plugin/dv_plugin_manager.h"
#include "modules/dreamview_plus/backend/updater/updater_manager.h"
/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

using apollo::dreamview::ChannelInfo;
using apollo::dreamview::DataHandlerConf;
using apollo::dreamview::DataHandlerInfo;
using Json = nlohmann::json;

/**
 * @class SocketManager
 * @brief This is a major component of the Simulation backend, which
 * manage all websockets to respond to all frontend request.
 */
class SocketManager {
 public:
  /**
   * @brief Constructor of SocketManager.
   */
  SocketManager(WebSocketHandler *websocket, UpdaterManager *updater_manager,
                DvPluginManager *dv_plugin_manager);
  /**
   * @brief Broadcast data handler conf for all connections
   * @param clear_channel_msg true means broadcast data handler
   * conf after clear channel msg.
   */
  void BrocastDataHandlerConf(bool clear_channel_msg = false);

 private:
  /**
   * @brief Register data handler to support subscribe data、unsubscribe data
   */
  void RegisterDataHandlers();
  // bool RegisterDataHandler(const DataHandlerInfo &data_handler_info);
  //  The conf stores data-proto-handler related relationship
  DataHandlerConf data_handler_base_conf_;
  DataHandlerConf data_handler_conf_;
  bool enabled_;
  // Used to obtain deduplication of all channels.
  std::map<std::string, int> complete_channel_count_;
  // Used to deduplicate channels under data handler info.
  std::map<std::string, int> data_handler_channel_count_;
  WebSocketHandler *websocket_ = nullptr;
  UpdaterManager *updater_manager_ = nullptr;
  DvPluginManager *dv_plugin_manager_ = nullptr;
  void RegisterMessageHandlers();
  /**
   * @brief Subscribe data handler to publish data message.
   * @param json Json data from frontend.
   */
  bool Subscribe(const Json &json);
  /**
   * @brief UnSubscribe data handler to publish data message.
   * @param json Json data from frontend.
   */
  bool UnSubscribe(const Json &json);
  bool GetDataUpdaterChannels(const std::string &updater_path,
                              std::vector<std::string> *channels);
  /**
   * @brief Update the chanel allowed by the corresponding updater manager.
   * @param updater_path The name of the updater manager.
   * @param channel_name The name of the new or reduced channel.
   * @param operation Identifies whether a channel is added or reduced.
   */
  bool ModifyUpdaterChannels(const std::string &updater_path,
                             const std::string &channel_name,
                             const std::string &operation);
  Json GetDataHandlerInfo();
  Json ClearDataHandlerChannelMsgs();
  /**
   * @brief When the topology of the channel changes, incremental data is passed
   * to the front end.
   * @param change_msg Topology change messages.
   */
  void RefreshChannels(const apollo::cyber::proto::ChangeMsg &change_msg);
  /**
   * @brief When the topology of the channel changes, incrementally update all
   * channels.
   * @param change_msg Topology change messages.
   * @param updater_info Messages passed to the front end.
   */
  bool UpdateCyberChannels(const apollo::cyber::proto::ChangeMsg &change_msg,
                           DataHandlerInfo &updater_info);
  /**
   * @brief When the topology of the channel changes, incrementally update the
   * channel under data handler info.
   * @param change_msg Topology change messages.
   * @param data_handler_conf_diff Messages passed to the front end.
   * @param flag Identifies whether there is an incremental channel.
   */
  void RefreshDataHandlerChannels(
      const apollo::cyber::proto::ChangeMsg &change_msg,
      DataHandlerConf &data_handler_conf_diff, bool &flag);
  /**
   * @brief Assemble added or subtracted channels into messages.
   * @param role_attr The role_attr of the transformed channel.
   * @param updater_info Messages passed to the front end.
   */
  bool UpdateUpdaterInfo(const apollo::cyber::proto::ChangeMsg &change_msg,
                         DataHandlerInfo &updater_info);
};

}  // namespace dreamview
}  // namespace apollo
