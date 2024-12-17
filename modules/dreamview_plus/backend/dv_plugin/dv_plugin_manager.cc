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

#include "modules/dreamview_plus/backend/dv_plugin/dv_plugin_manager.h"

namespace apollo {
namespace dreamview {

using apollo::cyber::plugin_manager::PluginManager;
using apollo::dreamview::DataHandlerConf;

void DvPluginManager::Init() {
  AutoLoadPlugins();
  GetPluginClassNames();
  CreatePluginInstances();
}

void DvPluginManager::Start() { RunInstances(); }

bool DvPluginManager::AutoLoadPlugins() {
  if (!PluginManager::Instance()->LoadInstalledPlugins()) {
    AERROR << "Load plugins failed!";
    return false;
  }
  AERROR << "pluginmanager index: " << PluginManager::Instance();
  return true;
}

void DvPluginManager::GetPluginClassNames() {
  derived_class_names_ =
      PluginManager::Instance()->GetDerivedClassNameByBaseClass<DvPluginBase>();
}

bool DvPluginManager::CreatePluginInstances() {
  if (derived_class_names_.empty()) {
    AERROR
        << "There be seems no dreamview_plus plugins, please install plugins.";
    return false;
  }
  for (const auto &class_name : derived_class_names_) {
    CreatePluginInstance(class_name);
  }
  return true;
}

bool DvPluginManager::CreatePluginInstance(const std::string class_name) {
  auto dv_plugin_ptr =
      PluginManager::Instance()->CreateInstance<DvPluginBase>(class_name);
  if (dv_plugin_ptr != nullptr) {
    dv_plugin_ptr->Init();
    auto websocket_handlers = dv_plugin_ptr->GetWebSocketHandlers();
    auto handlers = dv_plugin_ptr->GetHandlers();
    auto updater_websocket_handlers = dv_plugin_ptr->GetUpdaterHandlers();
    if (!websocket_handlers.empty()) {
      RegisterWebsocketHandlers(websocket_handlers);
    }
    if (!handlers.empty()) {
      RegisterHandlers(handlers);
    }
    if (!updater_websocket_handlers.empty()) {
      auto plugin_data_handler_conf = dv_plugin_ptr->GetDataHandlerConf();
      RegisterUpdaterHandlers(updater_websocket_handlers,
                              plugin_data_handler_conf);
    }
    plugin_instance_map_[class_name] = dv_plugin_ptr;
  }
  return true;
}

void DvPluginManager::RunInstances() {
  for (const auto &instance : plugin_instance_map_) {
    AINFO << "Instance: " << instance.first << " start running";
    instance.second->Run();
  }
}

void DvPluginManager::RegisterWebsocketHandlers(
    std::map<std::string, WebSocketHandler *> &websocket_handler_map) {
  for (auto &handler : websocket_handler_map) {
    server_->addWebSocketHandler(handler.first, *handler.second);
  }
}

void DvPluginManager::RegisterHandlers(
    std::map<std::string, CivetHandler *> &hander_map) {
  for (auto &handler : hander_map) {
    server_->addHandler(handler.first, *handler.second);
  }
}

void DvPluginManager::RegisterUpdaterHandlers(
    std::map<std::string, UpdaterBase *> &updater_handler_map,
    const DataHandlerConf &data_handler_conf) {
  // 1. register updater
  for (auto &handler : updater_handler_map) {
    AINFO << "RegisterUpdaterHandlers: " << handler.first;
    updater_manager_->RegisterUpdater(handler.first, handler.second);
  }
  // 2. register data handler info
  for (auto &handler : data_handler_conf.data_handler_info()) {
    if (data_handler_conf_.data_handler_info().find(handler.first) !=
        data_handler_conf_.data_handler_info().end()) {
      AERROR << "There are duplicate updater handlers between dv plugins";
      continue;
    }
    auto data_handler_info = data_handler_conf_.mutable_data_handler_info();
    (*data_handler_info)[handler.first] = handler.second;
  }
  AINFO << "RegisterUpdaterHandlers: data_handler_conf_: "
        << data_handler_conf_.DebugString();
}

void DvPluginManager::Stop() {
  for (const auto &instance : plugin_instance_map_) {
    AINFO << "Instance: " << instance.first << " stopped";
    instance.second->Stop();
  }
}

}  // namespace dreamview
}  // namespace apollo
