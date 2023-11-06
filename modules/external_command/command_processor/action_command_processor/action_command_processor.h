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
 * @file action_command_processor.h
 **/

#pragma once

#include <memory>
#include <string>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/pad_msg.pb.h"
#include "modules/common_msgs/external_command_msgs/action_command.pb.h"
#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/common_msgs/planning_msgs/pad_msg.pb.h"
#include "modules/external_command/command_processor/action_command_processor/proto/action_command_config.pb.h"

#include "cyber/cyber.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/external_command/command_processor/command_processor_base/command_processor_base.h"

namespace apollo {
namespace external_command {

class MessageReader;
class WriterHandle;

class ActionCommandProcessor : public CommandProcessorBase {
 public:
  bool Init(const std::shared_ptr<cyber::Node>& node) override;
  /**
   * @brief Get the command status.
   * @param command_id Id of the command.
   * @param status Output command status.
   */
  bool GetCommandStatus(int64_t command_id,
                        CommandStatus* status) const override;

 private:
  /**
   * @brief Process the incoming action command.
   * @param command Incoming action command.
   * @param status The command process result before sending to planning module.
   */
  void OnCommand(const std::shared_ptr<ActionCommand>& command,
                 std::shared_ptr<CommandStatus>& status);
  /**
   * @brief Switch driving mode to auto. Try several times and wait for the
   * driving mode changed to auto mode.
   * @param module_name Name of module who send this action message.
   */
  void SwitchToAutoMode(const std::string& module_name);
  /**
   * @brief Switch driving mode to manual. Try several times and wait for the
   * driving mode changed to auto mode.
   * @param module_name Name of module who send this action message.
   */
  void SwitchToManualMode(const std::string& module_name);
  /**
   * @brief Switch driving mode to the target mode.
   * @param driving_mode The target mode.
   * @param driving_action The driving action in the control message to be sent
   * for the target driving mode.
   * @param module_name Name of module who send this action message.
   * @return True if mode is switched successfully.
   */
  bool SwitchMode(const canbus::Chassis::DrivingMode& driving_mode,
                  const control::DrivingAction& driving_action,
                  const std::string& module_name);
  /**
   * @brief Check if the scenario is finished and update the command status.
   * @param scenario_name The scenario name.
   * @param status Output command status.
   */
  void CheckScenarioFinished(const std::string& scenario_name,
                             CommandStatus* status) const;
  /**
   * @brief Check if drived mode is switched and update the command status.
   * @param drive_mode The drive mode to be checked.
   * @param status Output command status.
   */
  void CheckModeSwitchFinished(
      const apollo::canbus::Chassis::DrivingMode& drive_mode,
      CommandStatus* status) const;
  /**
   * @brief Update the mode switch status.
   * @param message The message of mode switch.
   * @param is_error If mode switch failed.
   */
  void UpdateModeSwitchStatus(const std::string& message, bool is_error);

  std::shared_ptr<cyber::Service<ActionCommand, CommandStatus>>
      command_service_;
  std::shared_ptr<WriterHandle> planning_action_writer_;
  std::shared_ptr<WriterHandle> control_action_writer_;

  CommandStatus last_mode_switch_status_;
  std::shared_ptr<ActionCommand> last_command_;
  ActionCommandConfig special_config_;
  MessageReader* message_reader_;
  std::string chassis_status_name_;
  std::string planning_status_name_;
  std::string planning_command_status_name_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::external_command::ActionCommandProcessor, CommandProcessorBase)

}  // namespace external_command
}  // namespace apollo
