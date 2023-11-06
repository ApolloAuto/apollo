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
 * @file command_processor_base.h
 */

#pragma once

#include <memory>
#include <string>

#include "cyber/cyber.h"

/**
 * @namespace apollo::external_command
 * @brief apollo::external_command
 */
namespace apollo {
namespace external_command {

class CommandProcessorConfig;
class CommandStatus;

/**
 * @class ExternalCommandProcessComponent
 *
 * @brief The external interface for processing external commands.
 */
class CommandProcessorBase {
 public:
  CommandProcessorBase();

  virtual ~CommandProcessorBase() = default;

  virtual bool Init(const std::shared_ptr<cyber::Node>& node);
  /**
   * @brief Get the command status.
   * @param command_id Id of the command.
   * @param status Output command status.
   */
  virtual bool GetCommandStatus(int64_t command_id,
                                CommandStatus* status) const = 0;

 protected:
  const CommandProcessorConfig& GetProcessorConfig() const;

  const std::shared_ptr<cyber::Node>& Node() const;

 private:
  std::shared_ptr<CommandProcessorConfig> processor_config_;
  std::shared_ptr<cyber::Node> node_;
};

}  // namespace external_command
}  // namespace apollo
