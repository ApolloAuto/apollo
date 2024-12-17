/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file
 */

#include <map>
#include <memory>
#include <string>

#include "CivetServer.h"

#include "modules/dreamview_plus/proto/data_handler.pb.h"
#include "modules/dreamview/backend/common/handlers/websocket_handler.h"
#include "modules/dreamview_plus/backend/updater/updater_base.h"
namespace apollo {
namespace dreamview {

/**
 * @class DvPluginBase
 *
 * @brief A base class that a dreamview plus plug-in can inherit from.
 */
class DvPluginBase {
 public:
  DvPluginBase() {}

  /**
   * @brief Initialize the interface for instances inherited by derived classes.
   *
   * @param config_path Configuration file path required when initializing
   * multiple instances. Generally obtained through the plug-in configuration
   * file.
   */
  virtual void Init() = 0;

  /**
   * @brief The instance running interface that needs to be inherited by the
   * derived class mainly implements the calling of the instance.
   */
  virtual void Run() = 0;

  virtual ~DvPluginBase() = default;

  /**ß
   * @brief Get dreamview plus websocket handlers' info from base class or
   * derived class.
   * @return The return value is a map, the key value is uri, and the value is
   * the websocket handler corresponding to uri.
   */
  virtual std::map<std::string, WebSocketHandler *> GetWebSocketHandlers();

  /**
   * @brief Get dreamview plus http handlers' info from base class or
   * derived class.
   * @return The return value is a map, the key value is uri, and the value is
   * the handler corresponding to uri.
   */
  virtual std::map<std::string, CivetHandler *> GetHandlers();

  /**
   * @brief Get the websocket that needs to be subscribed and unsubscribed
   * @return The return value is a map, the key value is uri, and the value is
   * the handler and data handler info corresponding to uri.
   */
  virtual std::map<std::string, UpdaterBase *> GetUpdaterHandlers();

  /**
   * @brief Get the data handler info that needs to be subscribed and
   * unsubscribed
   * @return The data handler info of the plugin.
   */
  virtual apollo::dreamview::DataHandlerConf GetDataHandlerConf();

  /**
   * @brief Mainly stop some resources of the instance.
   */
  virtual void Stop();
};

}  // namespace dreamview
}  // namespace apollo
