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
#include "modules/dreamview/backend/common/plugins/plugin_manager.h"

#include <dirent.h>

#include <limits>
#include <string>

#include "google/protobuf/util/json_util.h"

#include "cyber/common/file.h"
#include "modules/common/util/json_util.h"
#include "modules/common/util/message_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

using apollo::common::util::FillHeader;
using apollo::common::util::JsonUtil;
using google::protobuf::util::JsonStringToMessage;
using Json = nlohmann::json;
using std::string;

// scenario_set: update one scenario set from studio to local
// scenarios: update all scenarios
namespace {
std::map<string, int> data_type_dict = {{
                                            "scenario_set", 0,
                                        },
                                        {
                                            "scenarios", 1,
                                        },
                                        {
                                            "dynamic_model", 2,
                                        },
                                        {
                                            "records", 3,
                                        },
                                        {
                                            "vehicles", 4,
                                        },
                                        {
                                            "maps", 5,
                                        }};
}  // namespace
namespace apollo {     // namespace apollo
namespace dreamview {  // namespace dreamview
PluginManager::PluginManager(WebSocketHandler* plugin_ws)
    : node_(cyber::CreateNode("PluginManager")),
      enabled_(false),
      plugin_ws_(plugin_ws) {
  RegisterDvSupportApis();
  RegisterPlugins();
}

void PluginManager::Start(DvCallback callback_api) {
  callback_api_ = callback_api;
  enabled_ = true;
  return;
}

void PluginManager::Stop() {
  if (enabled_) {
    std::string stop_command;
    for (auto iter = plugins_.begin(); iter != plugins_.end(); iter++) {
      stop_command = iter->second.stop_command;
      const int ret = std::system(stop_command.data());
      if (ret != 0) {
        AERROR << "Failed to stop plugin! ret: " << ret;
      }
    }
    plugins_.clear();
    // need kill all plugin process
  }
  enabled_ = false;
}

auto PluginManager::InitPluginReader(const ChannelConf& channel_conf,
                                     const string& channel_prefix,
                                     const string& plugin_name)
    -> std::shared_ptr<cyber::Reader<DvPluginMsg>> {
  if (!channel_conf.has_location()) {
    AERROR << "Failed to init reader for plugins for missing required file "
              "location";
    return nullptr;
  }
  const string channel_location = channel_conf.location();
  // Plugin related channel name should follow the plugin specification
  if (channel_location.find(channel_prefix) != 0) {
    AERROR << "Plugin related channel should observe channel name conventions!";
    return nullptr;
  }
  if (plugins_[plugin_name].readers.find(channel_location) !=
      plugins_[plugin_name].readers.end()) {
    AERROR << "Plugin has already register this channel: " << channel_location;
    return nullptr;
  }
  cyber::ReaderConfig reader_config;
  reader_config.channel_name = channel_location;
  reader_config.pending_queue_size = channel_conf.pending_queue_size();
  auto reader = node_->CreateReader<DvPluginMsg>(
      reader_config, [this](const std::shared_ptr<DvPluginMsg>& msg) {
        if (!ReceiveMsgFromPlugin(*msg)) {
          AERROR << "Failed to handle received msg from plugin";
        }
        return;
      });
  return reader;
};

auto PluginManager::InitPluginWriterAndMsg(const ChannelConf& channel_conf,
                                           const string& channel_prefix,
                                           const string& plugin_name)
    -> std::shared_ptr<cyber::Writer<DvPluginMsg>> {
  if (!channel_conf.has_location()) {
    AERROR << "Failed to init writer for plugins for missing required file "
              "location";
    return nullptr;
  }
  const string channel_location = channel_conf.location();
  // Plugin related channel name should follow the plugin specification
  if (channel_location.find(channel_prefix) != 0) {
    AERROR << "Plugin related channel should observe channel name conventions!";
    return nullptr;
  }
  if (plugins_[plugin_name].writers.find(channel_location) !=
      plugins_[plugin_name].writers.end()) {
    AERROR << "Plugin has already register this channel!";
    return nullptr;
  }
  auto writer = node_->CreateWriter<DvPluginMsg>(channel_location);
  if (!writer) {
    AERROR << "Failed to create writer!";
    return nullptr;
  }
  plugins_[plugin_name].writers[channel_location] = writer;
  if (channel_conf.support_msg_name_size()) {
    for (auto& support_msg : channel_conf.support_msg_name()) {
      if (plugins_[plugin_name].plugin_accept_msg.find(support_msg) !=
          plugins_[plugin_name].plugin_accept_msg.end()) {
        AERROR << "One-to-one message and channel, no repetition is allowed";
        return nullptr;
      }
      plugins_[plugin_name].plugin_accept_msg[support_msg] = channel_location;
    }
  }
  return writer;
};

bool PluginManager::RegisterPlugin(
    const std::shared_ptr<PluginConfig>& plugin_config) {
  if (!plugin_config->has_name() || !plugin_config->has_launch_command() ||
      !plugin_config->process_command_keywords_size() ||
      !plugin_config->has_stop_command()) {
    AERROR << "Failed to register plugin for required fields missing!";
    return false;
  }
  string plugin_name = plugin_config->name();
  string launch_command = plugin_config->launch_command();
  if (plugins_.find(plugin_name) != plugins_.end()) {
    AERROR << "This plugin has already registered! Don't install the plugin "
              "again!";
    return false;
  }
  struct PluginInfo plugin_info;
  plugins_[plugin_name] = {};
  plugins_[plugin_name].launch_command = launch_command;
  plugins_[plugin_name].stop_command = plugin_config->stop_command();
  string channel_prefix = FLAGS_plugin_channel_prefix + plugin_name + "/";
  for (auto reader_channel_conf : plugin_config->reader_channel_conf()) {
    auto plugin_reader =
        InitPluginReader(reader_channel_conf, channel_prefix, plugin_name);
    if (plugin_reader == nullptr) {
      AERROR << "Failed to register plugin reader!";
      plugins_.erase(plugin_name);
      return false;
    }
  }
  for (auto writer_channel_conf : plugin_config->writer_channel_conf()) {
    auto plugin_writer = InitPluginWriterAndMsg(writer_channel_conf,
                                                channel_prefix, plugin_name);
    if (plugin_writer == nullptr) {
      AERROR << "Failed to register plugin writer!";
      plugins_.erase(plugin_name);
      return false;
    }
  }
  const int ret = std::system(("nohup " + launch_command + " &").data());
  if (ret == 0) {
    AINFO << "SUCCESS to launch plugin: " << plugin_name;
  } else {
    AERROR << "Failed to launch plugin: " << plugin_name << " ret: " << ret;
    plugins_.erase(plugin_name);
    return false;
  }
  for (auto command_keyword : plugin_config->process_command_keywords()) {
    plugins_[plugin_name].process_command_keywords.push_back(command_keyword);
  }
  return true;
}

bool PluginManager::RegisterPlugins() {
  const std::string plugin_path =
      (cyber::common::GetEnv("HOME")) + FLAGS_plugin_path;
  DIR* directory = opendir(plugin_path.c_str());
  if (directory == nullptr) {
    AERROR << "Cannot open directory " << FLAGS_plugin_path;
    return false;
  }
  struct dirent* entry;
  bool register_res = true;
  while ((entry = readdir(directory)) != nullptr && register_res) {
    // skip directory_path/. and directory_path/..
    if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) {
      continue;
    }
    if (entry->d_type != DT_DIR) {
      // skip not directory
      continue;
    }
    const string plugin_config_file_name =
        entry->d_name + FLAGS_plugin_config_file_name_suffix;
    const string plugin_config_file_path =
        (cyber::common::GetEnv("HOME")) + FLAGS_plugin_path + "/" +
        entry->d_name + "/" + plugin_config_file_name;
    if (!cyber::common::PathExists(plugin_config_file_path)) {
      AERROR << "Cannot find plugin:" << entry->d_name
             << " plugin config file, jump it!";
      continue;
    }
    auto plugin_config = std::make_shared<PluginConfig>();
    if (!cyber::common::GetProtoFromFile(plugin_config_file_path,
                                         plugin_config.get())) {
      AWARN << "Unable to read plugin config from file: "
            << plugin_config_file_path;
      return false;
    }
    register_res = RegisterPlugin(plugin_config);
  }
  return register_res;
}

bool PluginManager::CheckPluginStatus(const string& plugin_name) {
  if (plugins_.find(plugin_name) == plugins_.end()) {
    AERROR << "Failed to register this plugin, cann't check!";
    return false;
  }
  // todo: Extract the logic for monitoring plugin status
  std::vector<string> running_processes;
  for (const auto& cmd_file : cyber::common::Glob("/proc/*/cmdline")) {
    // Get process command string.
    string cmd_string;
    if (cyber::common::GetContent(cmd_file, &cmd_string) &&
        !cmd_string.empty()) {
      // In /proc/<PID>/cmdline, the parts are separated with \0, which will be
      // converted back to whitespaces here.
      std::replace(cmd_string.begin(), cmd_string.end(), '\0', ' ');
      running_processes.push_back(cmd_string);
    }
  }
  bool command_found = false;
  for (const string& command : running_processes) {
    bool all_keywords_matched = true;
    for (const string& keyword :
         plugins_[plugin_name].process_command_keywords) {
      all_keywords_matched &= (command.find(keyword) != string::npos);
      if (!all_keywords_matched) {
        break;
      }
    }
    command_found |= all_keywords_matched;
  }
  if (!command_found) {
    AERROR << "Failed to pass plugin status check!";
    return false;
  }
  // Process command keywords are all matched. The process is running.
  return true;
}

bool PluginManager::SendMsgToPlugin(const string& json_str) {
  auto plugin_msg = std::make_shared<DvPluginMsg>();
  if (!JsonStringToMessage(json_str, plugin_msg.get()).ok()) {
    AERROR << "Failed to parse DvPluginMsg from json!";
    return false;
  }
  // Cancel check requestId for dv_plus does not have this field
  if (!plugin_msg->has_target() || !plugin_msg->has_name()) {
    AERROR << "Missing required field for DvPluginMsg.";
    return false;
  }
  const string plugin_name = plugin_msg->target();
  if (!CheckPluginStatus(plugin_name)) {
    return false;
  }
  const string msg_name = plugin_msg->name();

  if (plugins_[plugin_name].plugin_accept_msg.find(msg_name) ==
      plugins_[plugin_name].plugin_accept_msg.end()) {
    AERROR << "Plugin not accept this msg!";
    return false;
  }
  const string channel_location =
      plugins_[plugin_name].plugin_accept_msg[msg_name];
  if (plugins_[plugin_name].writers.find(channel_location) ==
      plugins_[plugin_name].writers.end()) {
    AERROR << "The plugin does not support communication on this channel";
    return false;
  }
  FillHeader("PluginManager", plugin_msg.get());
  plugins_[plugin_name].writers[channel_location]->Write(plugin_msg);
  return true;
}

void PluginManager::RegisterDvSupportApi(const string& api_name,
                                         const DvApi& api) {
  dv_support_apis_[api_name] = api;
}

void PluginManager::RegisterDvSupportApis() {
  RegisterDvSupportApi("UpdateScenarioSetList", &PluginManager::UpdateData);
  RegisterDvSupportApi("UpdateDynamicModelList", &PluginManager::UpdateData);
  RegisterDvSupportApi("UpdateRecordToStatus", &PluginManager::UpdateData);
  RegisterDvSupportApi("ResetVehicleConfigSuccess", &PluginManager::UpdateData);
  RegisterDvSupportApi("RefreshVehicleConfigSuccess",
                       &PluginManager::UpdateData);
  RegisterDvSupportApi("UpdateMapToStatus", &PluginManager::UpdateData);
}

bool PluginManager::ReceiveMsgFromPlugin(const DvPluginMsg& msg) {
  if (!msg.has_name()) {
    AERROR << "Invalid message name!";
    return false;
  }
  const string msg_name = msg.name();
  if (dv_support_apis_.find(msg_name) != dv_support_apis_.end()) {
    string json_res;
    bool result = (this->*dv_support_apis_[msg_name])(msg, json_res);
    if (!result) {
      AERROR << "Failed to handle msg!";
      return false;
    }
  }
  // Encapsulated as dreamview response message format
  Json response = JsonUtil::ProtoToTypedJson("PluginResponse", msg);
  response["action"] = "response";
  Json info = Json::parse(msg.info());
  response["data"]["info"] = info;
  bool broadcast;
  if (!JsonUtil::GetBooleanByPath(response, "data.broadcast", &broadcast)) {
    // default true,broadcast to websocket
    broadcast = true;
  }
  if (broadcast) {
    plugin_ws_->BroadcastData(response.dump());
  }
  return true;
}

bool PluginManager::UpdateData(const DvPluginMsg& msg, const string& json_str) {
  if (!msg.has_info()) {
    AERROR << "Failed to get data type!";
    return false;
  }
  const string info_str = msg.info();
  Json info({});
  info = Json::parse(info_str);
  if (!info.contains("data") || !info["data"].contains("data_type")) {
    AERROR << "Failed to get data or data type!";
    return false;
  }
  const string data_type = info["data"]["data_type"];
  if (data_type_dict.find(data_type) == data_type_dict.end()) {
    AERROR << "Dv don't support this kind of data type!";
    return false;
  }
  const int data_type_index = data_type_dict[data_type];
  bool update_data_res = false;
  switch (data_type_index) {
    case 0: {
      update_data_res = callback_api_("UpdateScenarioSetToStatus", info);
      break;
    }
    case 2: {
      // 下载成功-新增文件+register+本地hmistatus
      // 删除-删除文件+unregister+本地Hmistatus
      update_data_res = callback_api_("UpdateDynamicModelToStatus", info);
      break;
    }
    case 3: {
      update_data_res = callback_api_("UpdateRecordToStatus", info);
      break;
    }
    case 4: {
      update_data_res = callback_api_("UpdateVehicleToStatus", info);
      break;
    }
    case 5: {
      update_data_res = callback_api_("UpdateMapToStatus", info);
      break;
    }
    default:
      break;
  }
  if (!update_data_res) {
    AERROR << "Failed to update data!";
    return false;
  }
  return true;
}

}  // namespace dreamview
}  // namespace apollo
