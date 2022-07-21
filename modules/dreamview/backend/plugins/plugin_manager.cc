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
#include "modules/dreamview/backend/plugins/plugin_manager.h"

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
                                            "scenario_set",
                                            0,
                                        },
                                        {
                                            "scenarios",
                                            1,
                                        }};
}
namespace apollo {
namespace dreamview {
PluginManager::PluginManager(WebSocketHandler* plugin_ws)
    : node_(cyber::CreateNode("PluginManager")),
      enabled_(false),
      plugin_ws_(plugin_ws) {
  RegisterDvSupportApis();
  RegisterPlugins();
}

// todo: 注册好等于已经启动pluginmanager 这个地方是否有必要
void PluginManager::Start(DvCallback callback_api) {
  callback_api_ = callback_api;
  enabled_ = true;
  return;
}

void PluginManager::Stop() {
  if (enabled_) {
    // 清除reader writer 和字典
    std::string stop_command;
    // 结束dv的时候同时通过PluginManager关闭所有dv相关的插件
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

auto PluginManager::InitPluginReader(ChannelConf& channel_conf,
                                     const string& channel_prefix,
                                     const string& plugin_name)
    -> std::shared_ptr<cyber::Reader<DvPluginMsg>> {
  if (!channel_conf.has_location()) {
    AERROR << "Failed to init reader for plugins for missing required file "
              "location";
    return nullptr;
  }
  const string channel_location = channel_conf.location();
  // 判断channel是否遵循规范
  if (channel_location.find(channel_prefix) != 0) {
    AERROR << "Plugin related channel should observe channel name conventions!";
    return nullptr;
  }
  // 判断对应channel是否已经在字典中建立
  // 检查之前得先插入！！！不行再回退删掉
  if (plugins_[plugin_name].readers.find(channel_location) !=
      plugins_[plugin_name].readers.end()) {
    AERROR << "Plugin has already register this channel: " << channel_location;
    return nullptr;
  }
  // 建立reader writer 直接返回reader writer
  cyber::ReaderConfig reader_config;
  reader_config.channel_name = channel_location;
  reader_config.pending_queue_size = channel_conf.pending_queue_size();
  auto reader = node_->CreateReader<DvPluginMsg>(
      reader_config, [this](const std::shared_ptr<DvPluginMsg>& msg) {
        // 统一reader处理函数
        // Todo: add 统一处理函数
        if (!ReceiveMsgFromPlugin(*msg)) {
          AERROR << "Failed to handle received msg from plugin";
        }
        return;
      });
  return reader;
};

auto PluginManager::InitPluginWriterAndMsg(ChannelConf& channel_conf,
                                           const string& channel_prefix,
                                           const string& plugin_name)
    -> std::shared_ptr<cyber::Writer<DvPluginMsg>> {
  if (!channel_conf.has_location()) {
    AERROR << "Failed to init writer for plugins for missing required file "
              "location";
    return nullptr;
  }
  const string channel_location = channel_conf.location();
  // 判断channel是否遵循规范
  if (channel_location.find(channel_prefix) != 0) {
    AERROR << "Plugin related channel should observe channel name conventions!";
    return nullptr;
  }
  // 判断对应channel是否已经在字典中建立
  // 检查之前得先插入！！！不行再回退删掉
  if (plugins_[plugin_name].writers.find(channel_location) !=
      plugins_[plugin_name].writers.end()) {
    AERROR << "Plugin has already register this channel!";
    return nullptr;
  }
  // 建立reader writer 直接返回reader writer
  auto writer = node_->CreateWriter<DvPluginMsg>(channel_location);
  if (!writer) {
    AERROR << "Failed to create writer!";
    return nullptr;
  }
  // 建立writer和channel的联系
  plugins_[plugin_name].writers[channel_location] = writer;
  if (channel_conf.support_msg_name_size()) {
    for (auto& support_msg : channel_conf.support_msg_name()) {
      // string support_msg = channel_conf->support_msg_name();
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
  // 解析PluginConfig到字典
  // 注意校验不能重复!
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
  // 直接改为校验命令执行结果
  // 出问题了记得随时撤销数据
  plugins_[plugin_name] = {};
  plugins_[plugin_name].launch_command = launch_command;
  plugins_[plugin_name].stop_command = plugin_config->stop_command();
  // Register reader,writers
  // channel不能重复 一个channel 绑定一个reader 一个writer
  // 先用一个字典维护reader writer建立的channel
  // 对channel前缀做检查！
  // 所有定义的规范本身要检查！一定要确认
  string channel_prefix = FLAGS_plugin_channel_prefix + plugin_name + "/";
  for (auto reader_channel_conf : plugin_config->reader_channel_conf()) {
    auto plugin_reader =
        InitPluginReader(reader_channel_conf, channel_prefix, plugin_name);
    if (plugin_reader == nullptr) {
      AERROR << "Failed to register plugin reader!";
      // todo:对于每次return false终止判断的情况多考虑多学习！
      plugins_.erase(plugin_name);
      return false;
    }
  }
  for (auto writer_channel_conf : plugin_config->writer_channel_conf()) {
    auto plugin_writer = InitPluginWriterAndMsg(writer_channel_conf,
                                                channel_prefix, plugin_name);
    if (plugin_writer == nullptr) {
      AERROR << "Failed to register plugin writer!";
      // todo:对于每次return false终止判断的情况多考虑多学习！
      plugins_.erase(plugin_name);
      return false;
    }
  }
  // 注册结束 启动插件 启动失败则抹除这条记录
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
  // 根据gflag file 遍历文件夹
  // FLAGS_plugin_path
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
    // 判断Plugin Config是否存在！
    if (!cyber::common::PathExists(plugin_config_file_path)) {
      AERROR << "Cannot find plugin:" << entry->d_name
             << " plugin config file, jump it!";
      continue;
    }
    // 解析PluginConfig格式
    auto plugin_config = std::make_shared<PluginConfig>();
    if (!cyber::common::GetProtoFromFile(plugin_config_file_path,
                                         plugin_config.get())) {
      AWARN << "Unable to read plugin config from file: "
            << plugin_config_file_path;
      return false;
    }
    // 加载plugin到字典
    register_res = RegisterPlugin(plugin_config);
  }
  return register_res;
}

bool PluginManager::CheckPluginStatus(const string& plugin_name) {
  if (plugins_.find(plugin_name) == plugins_.end()) {
    AERROR << "Failed to register this plugin, cann't check!";
    return false;
  }
  // todo: 是否抽出来
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
  // 针对这种component+cyber
  // launch的启动进程，看看他在linux的启动命令，然后选用参数 todo: add check
  // command
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
  // 检查必须字段
  if (!plugin_msg->has_target() || !plugin_msg->has_name()) {
    AERROR << "Missing required field for DvPluginMsg.";
    return false;
  }
  const string plugin_name = plugin_msg->target();
  // check 插件状态
  if (!CheckPluginStatus(plugin_name)) {
    return false;
  }
  // plugin 状态ok 根据对应Target+msg_name 获取对应channel
  // Todo:@liangjingping 这个地方基本写的是固定。fe->be一定走这个逻辑
  // 一定需要发消息走channel给插件 认为这个通信一定存在
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
}

bool PluginManager::ReceiveMsgFromPlugin(const DvPluginMsg& msg) {
  // 解析Msg_name 并且看看dv后端有无注册对应api
  if (!msg.has_name()) {
    return false;
  }
  const string msg_name = msg.name();
  if (dv_support_apis_.find(msg_name) != dv_support_apis_.end()) {
    // 调用support api
    string json_res;
    bool result = (this->*dv_support_apis_[msg_name])(msg, json_res);
    if (!result) {
      AERROR << "Failed to handle msg!";
      return false;
    }
  }
  // 构造发送给前端的消息
  Json response = JsonUtil::ProtoToTypedJson("PluginMsg", msg);
  plugin_ws_->BroadcastData(response.dump());
  return true;
}

bool PluginManager::UpdateData(const DvPluginMsg& msg, string& json_str) {
  // 获取type
  // string 转json
  if (!msg.has_info()) {
    AERROR << "Failed to get data type!";
    return false;
  }
  const string info_str = msg.info();
  Json info;
  info = Json::parse(info_str);
  if (!info.contains("data_type")) {
    AERROR << "Failed to get data type!";
    return false;
  }
  const string data_type = info["data_type"];
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
    // case 1:
    // update_data_res = updateScenarios();
    //  break;
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