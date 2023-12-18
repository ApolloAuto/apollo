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

#pragma once

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/dreamview/proto/dv_plugin_msg.pb.h"
#include "modules/dreamview/proto/plugin_config.pb.h"

#include "cyber/cyber.h"
#include "modules/dreamview/backend/common/handlers/websocket_handler.h"

using apollo::dreamview::ChannelConf;
using apollo::dreamview::DvPluginMsg;
using apollo::dreamview::PluginConfig;

namespace apollo {
namespace dreamview {
struct PluginInfo {
  std::string launch_command;
  std::string stop_command;
  // for writer
  std::map<std::string, std::string> plugin_accept_msg;
  std::vector<std::string> process_command_keywords;
  // writer 指针 根据channel调用对应writer 去写信息
  std::map<std::string, std::shared_ptr<cyber::Writer<DvPluginMsg>>> writers;
  std::map<std::string, std::shared_ptr<cyber::Reader<DvPluginMsg>>> readers;
};
class PluginManager {
 public:
  /**
   * @class PluginManager
   *
   * @brief A module that maintains all dv-related plugin information.
   * Process the information communicated by the dv and the plugin.
   */
  using DvApi = bool (PluginManager::*)(const DvPluginMsg& msg,
                                        const std::string& json_str);
  using DvCallback = std::function<bool(const std::string& function_name,
                                        const nlohmann::json& param_json)>;

  explicit PluginManager(WebSocketHandler* plugin_ws);

  void Start(DvCallback callback_api);
  void Stop();

  bool IsEnabled() const { return true; }
  bool SendMsgToPlugin(const std::string& json_str);

 private:
  bool RegisterPlugins();
  bool RegisterPlugin(const std::shared_ptr<PluginConfig>& plugin_config);
  auto InitPluginReader(const ChannelConf& channel_conf,
                        const std::string& channel_prefix,
                        const std::string& plugin_name)
      -> std::shared_ptr<cyber::Reader<DvPluginMsg>>;
  auto InitPluginWriterAndMsg(const ChannelConf& channel_conf,
                              const std::string& channel_prefix,
                              const std::string& plugin_name)
      -> std::shared_ptr<cyber::Writer<DvPluginMsg>>;
  bool CheckPluginStatus(const std::string& plugin_name);
  void RegisterDvSupportApi(const std::string& api_name, const DvApi& api);
  void RegisterDvSupportApis();
  bool ReceiveMsgFromPlugin(const DvPluginMsg& msg);
  bool UpdateData(const DvPluginMsg& msg, const std::string& json_str);
  std::map<std::string, PluginInfo> plugins_;
  std::unique_ptr<cyber::Node> node_;
  bool enabled_;
  WebSocketHandler* plugin_ws_ = nullptr;
  std::map<std::string, DvApi> dv_support_apis_;
  DvCallback callback_api_;
};

}  // namespace dreamview
}  // namespace apollo
