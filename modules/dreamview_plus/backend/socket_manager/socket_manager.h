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
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

#include "modules/dreamview_plus/proto/data_handler.pb.h"

#include "cyber/common/log.h"
#include "modules/dreamview_plus/backend/handlers/websocket_handler.h"
#include "modules/dreamview_plus/backend/updater/updater_manager.h"
/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

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
  SocketManager(WebSocketHandler *websocket, UpdaterManager *updater_manager);
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
  WebSocketHandler *websocket_ = nullptr;
  UpdaterManager *updater_manager_ = nullptr;
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
  Json GetDataHandlerInfo();
  Json ClearDataHandlerChannelMsgs();
};

}  // namespace dreamview
}  // namespace apollo
