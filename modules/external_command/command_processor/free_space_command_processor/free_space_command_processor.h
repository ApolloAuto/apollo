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
 * @file free_space_command_processor.h
 **/

#pragma once

#include <memory>

#include "modules/common_msgs/external_command_msgs/free_space_command.pb.h"
#include "cyber/cyber.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/external_command/command_processor/command_processor_base/motion_command_processor_base.h"

namespace apollo {
namespace external_command {

class FreeSpaceCommandProcessor : public MotionCommandProcessorBase<FreeSpaceCommand> {
private:
    /**
     * @brief Convert moving command to RoutingRequest.
     * @param command moving command to be converted.
     * @param routing_request Convert result of RoutingRequest. If there is no
     * LaneFollow action in moving command, the convert result can be nullptr.
     * @return Return true if there is error occurs for converting.
     */
    bool Convert(
            const std::shared_ptr<FreeSpaceCommand>& command,
            std::shared_ptr<apollo::routing::RoutingRequest>& routing_request) const override;
    /**
     * @brief Process special command except RoutingRequest.
     * @param command RoutingCommand to be converted.
     * @param planning_command Output process result.
     * @return True if no error occurs.
     */
    bool ProcessSpecialCommand(
            const std::shared_ptr<FreeSpaceCommand>& command,
            const std::shared_ptr<apollo::planning::PlanningCommand>& planning_command) const override;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::external_command::FreeSpaceCommandProcessor, CommandProcessorBase)

}  // namespace external_command
}  // namespace apollo
