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
 * @file external_command_process_component.cc
 */

#include "modules/external_command/process_component/external_command_process_component.h"

#include "modules/external_command/process_component/proto/process_component_config.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/external_command/command_processor/command_processor_base/command_processor_base.h"

/**
 * @namespace apollo::external_command
 * @brief apollo::external_command
 */
namespace apollo {
namespace external_command {

bool ExternalCommandProcessComponent::Init() {
  // Load the external command processors according to the config.
  ProcessComponentConfig config;
  if (!GetProtoConfig(&config)) {
    AERROR << "Unable to load ExternalCommandProcessComponent conf file: "
           << ConfigFilePath();
    return false;
  }
  const auto& plugin_manager = cyber::plugin_manager::PluginManager::Instance();
  for (const auto& processor_class_name : config.processor()) {
    command_processors_.emplace_back(
        plugin_manager->CreateInstance<CommandProcessorBase>(
            processor_class_name));
    command_processors_.back()->Init(node_);
  }
  command_status_service_ =
      node_->CreateService<CommandStatusRequest, CommandStatus>(
          config.output_command_status_name(),
          [this](const std::shared_ptr<CommandStatusRequest>& request,
                 std::shared_ptr<CommandStatus>& response) {
            bool is_get_status = false;
            // Get the command status from command processors.
            for (const auto& processor : command_processors_) {
              if (processor->GetCommandStatus(request->command_id(),
                                              response.get())) {
                is_get_status = true;
                break;
              }
            }
            if (!is_get_status) {
              response->set_status(CommandStatusType::UNKNOWN);
              response->set_message("Cannot get the status of command.");
            }
          });
  AINFO << "ExternalCommandProcessComponent init finished.";
  return true;
}

}  // namespace external_command
}  // namespace apollo
