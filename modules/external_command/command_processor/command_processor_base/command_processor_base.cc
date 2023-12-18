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
 * @file command_processor_base.cc
 */

#include "modules/external_command/command_processor/command_processor_base/command_processor_base.h"

#include <cxxabi.h>

#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/external_command/command_processor/command_processor_base/proto/command_processor_config.pb.h"

#include "cyber/common/file.h"
#include "cyber/plugin_manager/plugin_manager.h"

/**
 * @namespace apollo::external_command
 * @brief apollo::external_command
 */
namespace apollo {
namespace external_command {

CommandProcessorBase::CommandProcessorBase()
    : processor_config_(std::make_shared<CommandProcessorConfig>()) {}

bool CommandProcessorBase::Init(const std::shared_ptr<cyber::Node>& node) {
  int status;
  std::string class_name =
      abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);
  // Generate the default task config path from PluginManager.
  std::string config_path =
      apollo::cyber::plugin_manager::PluginManager::Instance()
          ->GetPluginConfPath<CommandProcessorBase>(class_name,
                                                    "conf/config.pb.txt");
  if (!cyber::common::GetProtoFromFile(config_path, processor_config_.get())) {
    AERROR << "Cannot get config of " << class_name;
    return false;
  }
  node_ = node;
  return true;
}

const CommandProcessorConfig& CommandProcessorBase::GetProcessorConfig() const {
  return *processor_config_;
}

const std::shared_ptr<cyber::Node>& CommandProcessorBase::Node() const {
  return node_;
}

}  // namespace external_command
}  // namespace apollo
