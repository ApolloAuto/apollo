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

#include "modules/dreamview_plus/backend/dv_plugin/dv_plugin_base.h"

namespace apollo {
namespace dreamview {

std::map<std::string, WebSocketHandler *> DvPluginBase::GetWebSocketHandlers() {
  return std::map<std::string, WebSocketHandler *>();
}

std::map<std::string, CivetHandler *> DvPluginBase::GetHandlers() {
  return std::map<std::string, CivetHandler *>();
}

std::map<std::string, UpdaterBase *> DvPluginBase::GetUpdaterHandlers() {
  return std::map<std::string, UpdaterBase *>();
}

apollo::dreamview::DataHandlerConf DvPluginBase::GetDataHandlerConf() {
  return apollo::dreamview::DataHandlerConf();
}

void DvPluginBase::Stop() { return; }

}  // namespace dreamview
}  // namespace apollo
