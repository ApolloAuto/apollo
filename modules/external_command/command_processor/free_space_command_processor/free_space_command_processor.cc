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
 * @file free_space_command_processor.cc
 **/

#include "modules/external_command/command_processor/free_space_command_processor/free_space_command_processor.h"

namespace apollo {
namespace external_command {

bool FreeSpaceCommandProcessor::Convert(
        const std::shared_ptr<FreeSpaceCommand>& command,
        std::shared_ptr<apollo::routing::RoutingRequest>& routing_request) const {
    routing_request = nullptr;
    return true;
}

bool FreeSpaceCommandProcessor::ProcessSpecialCommand(
        const std::shared_ptr<FreeSpaceCommand>& command,
        const std::shared_ptr<apollo::planning::PlanningCommand>& planning_command) const {
    auto custom_command = planning_command->mutable_custom_command();
    custom_command->PackFrom(*command);
    return true;
}

}  // namespace external_command
}  // namespace apollo
