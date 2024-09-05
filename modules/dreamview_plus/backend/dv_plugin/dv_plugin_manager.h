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

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/dreamview_plus/proto/data_handler.pb.h"
#include "cyber/common/macros.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/dreamview_plus/backend/dv_plugin/dv_plugin_base.h"
#include "modules/dreamview_plus/backend/updater/updater_manager.h"

namespace apollo {
namespace dreamview {

/**
 * @class DvPluginManager
 *
 * @brief A class that manages dreamview plus plug-ins.
 */
class DvPluginManager {
 public:
  explicit DvPluginManager(CivetServer* server, UpdaterManager* updater_manager)
      : server_(server), updater_manager_(updater_manager) {}

  /**
   * @brief Method to initialize the plug-in management class.
   */
  void Init();

  /**
   * @brief Methods to run the plug-in management class.
   */
  void Start();

  /**
   * @brief Methods to stop the resource of plug-in management class.
   */
  void Stop();

  apollo::dreamview::DataHandlerConf data_handler_conf_;

 private:
  void GetPluginClassNames();
  bool AutoLoadPlugins();
  bool CreatePluginInstances();

  /**
   * @brief Create a plugin instance based on the configuration file.
   *
   * @param class_name The class name of the plugin instance.
   * @param config_path The path to the plug-in instance configuration file.
   */
  bool CreatePluginInstance(const std::string class_name);
  void RunInstances();
  void RegisterWebsocketHandlers(
      std::map<std::string, WebSocketHandler*>& websocket_handler_map);
  void RegisterHandlers(std::map<std::string, CivetHandler*>& hander_map);
  void RegisterUpdaterHandlers(
      std::map<std::string, UpdaterBase*>& updater_handler_map,
      const apollo::dreamview::DataHandlerConf& data_hander_info);
  std::vector<std::string> derived_class_names_;
  std::map<std::string, std::shared_ptr<DvPluginBase>> plugin_instance_map_;
  CivetServer* server_ = nullptr;
  UpdaterManager* updater_manager_ = nullptr;
  DECLARE_SINGLETON(DvPluginManager);
};

}  // namespace dreamview
}  // namespace apollo
