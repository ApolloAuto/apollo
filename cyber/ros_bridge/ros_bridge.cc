/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <vector>

#include "cyber/ros_bridge/common/macros.h"

#include "cyber/ros_bridge/proto/ros_bridge_conf.pb.h"
#include "cyber/cyber.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "cyber/ros_bridge/common/bridge_argument.h"
#include "cyber/ros_bridge/common/ros_bridge_gflags.h"
#include "cyber/ros_bridge/common/utils.h"
#include "cyber/ros_bridge/converter_base/message_converter.h"

#include "gperftools/heap-profiler.h"
#include "gperftools/malloc_extension.h"
#include "gperftools/profiler.h"

#if __has_include("rclcpp/rclcpp.hpp")
#include "rclcpp/rclcpp.hpp"
#endif

using apollo::cyber::BridgeArgument;
using apollo::cyber::MessageConverter;
using apollo::cyber::plugin_manager::PluginManager;
using namespace apollo::cyber::common;  // NOLINT

int main(int argc, char** argv) {
  apollo::cyber::BridgeConf bridge_conf;
  std::vector<std::shared_ptr<MessageConverter>> converter_list_;

  BridgeArgument bridge_args;
  bridge_args.ParseArgument(argc, argv);

  apollo::cyber::Init(argv[0]);

#ifdef RCLCPP__RCLCPP_HPP_
  rclcpp::init(argc, argv);
#endif

  PluginManager::Instance()->LoadInstalledPlugins();

  auto config_path =
      GetAbsolutePath(WorkRoot(), apollo::cyber::FLAGS_bridge_conf_path);
  if (!GetProtoFromFile(config_path, &bridge_conf)) {
    AERROR << "parse ros bridge config failed!";
    return 1;
  }

  for (int i = 0; i < bridge_conf.converter_size(); i++) {
    auto converter =
        PluginManager::Instance()->CreateInstance<MessageConverter>(
            apollo::cyber::GetFullConverterClassName(
                bridge_conf.converter(i).type()));
    ACHECK(converter->Init())
        << "Can not init converter " << bridge_conf.converter(i).type();
    converter_list_.push_back(converter);
  }

  if (bridge_args.GetEnableCpuprofile()) {
    auto profile_filename = bridge_args.GetProfileFilename();
    ProfilerStart(profile_filename.c_str());
  }

  if (bridge_args.GetEnableHeapprofile()) {
    auto profile_filename = bridge_args.GetHeapProfileFilename();
    HeapProfilerStart(profile_filename.c_str());
  }

  apollo::cyber::WaitForShutdown();
#ifdef RCLCPP__RCLCPP_HPP_
  rclcpp::shutdown();
#endif

  if (bridge_args.GetEnableCpuprofile()) {
    ProfilerStop();
  }

  if (bridge_args.GetEnableHeapprofile()) {
    HeapProfilerDump("Befor shutdown");
    HeapProfilerStop();
  }

  return 0;
}
