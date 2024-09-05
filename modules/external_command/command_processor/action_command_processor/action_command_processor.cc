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
 * @file action_command_processor.cc
 **/

#include "modules/external_command/command_processor/action_command_processor/action_command_processor.h"

#include <cxxabi.h>

#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/external_command/command_processor/command_processor_base/proto/command_processor_config.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/util/message_util.h"
#include "modules/external_command/command_processor/command_processor_base/util/message_reader.h"
#include "modules/external_command/command_processor/command_processor_base/util/message_writer.h"

namespace apollo {
namespace external_command {

bool ActionCommandProcessor::Init(const std::shared_ptr<cyber::Node>& node) {
  if (!CommandProcessorBase::Init(node)) {
    return false;
  }
  message_reader_ = MessageReader::Instance();
  const auto& config = GetProcessorConfig();
  // Get special config for ActionCommandProcessor.
  int status = 0;
  std::string class_name =
      abi::__cxa_demangle(typeid(*this).name(), nullptr, nullptr, &status);
  std::string special_config_path =
      cyber::plugin_manager::PluginManager::Instance()
          ->GetPluginConfPath<CommandProcessorBase>(
              class_name, "conf/special_config.pb.txt");
  if (!cyber::common::LoadConfig<ActionCommandConfig>(special_config_path,
                                                      &special_config_)) {
    AERROR << "Cannot get special config of ActionCommandProcessor";
    return false;
  }
  // Create service for input command.
  command_service_ = node->CreateService<ActionCommand, CommandStatus>(
      config.input_command_name(),
      [this](const std::shared_ptr<ActionCommand>& command,
             std::shared_ptr<CommandStatus>& status) {
        this->OnCommand(command, status);
      });
  // Create writers for output command.
  CHECK_GT(config.output_command_name().size(), 1);
  auto message_writer = MessageWriter::Instance();
  planning_action_writer_ =
      message_writer->RegisterMessage<apollo::planning::PadMessage>(
          config.output_command_name().Get(0));
  control_action_writer_ =
      message_writer->RegisterMessage<apollo::control::PadMessage>(
          config.output_command_name().Get(1));
  // Create reader for input command status.
  CHECK_GT(config.input_command_status_name().size(), 2);

  chassis_status_name_ = config.input_command_status_name().Get(0);
  planning_status_name_ = config.input_command_status_name().Get(1);
  planning_command_status_name_ = config.input_command_status_name().Get(2);
  message_reader_->RegisterMessage<apollo::canbus::Chassis>(
      chassis_status_name_);
  message_reader_->RegisterMessage<apollo::planning::ADCTrajectory>(
      planning_status_name_);
  message_reader_->RegisterMessage<CommandStatus>(
      planning_command_status_name_);
  return true;
}

bool ActionCommandProcessor::GetCommandStatus(int64_t command_id,
                                              CommandStatus* status) const {
  CHECK_NOTNULL(status);
  if (nullptr == last_command_) {
    return false;
  }
  if (last_command_->command_id() != command_id) {
    return false;
  }
  std::string module_name = "UNKNOWN";
  if (last_command_->has_header()) {
    module_name = last_command_->header().module_name();
  }
  common::util::FillHeader(module_name, status);
  status->set_command_id(command_id);
  switch (last_command_->command()) {
    // Get status of "FOLLOW", "CHANGE_LEFT", "CHANGE_RIGHT".
    case external_command::ActionCommandType::FOLLOW:
    case external_command::ActionCommandType::CHANGE_LEFT:
    case external_command::ActionCommandType::CHANGE_RIGHT: {
      // todo: Check the real planning status and set command status.
      status->set_status(apollo::external_command::CommandStatusType::ERROR);
      status->set_message("Cannot execute the command!");
    } break;
    // Get status of "PULL_OVER".
    case external_command::ActionCommandType::PULL_OVER: {
      // When planning is in PullOverScenario and the scenario is finished,
      // command status is finished.
      CheckScenarioFinished(special_config_.pull_over_scenario_name(), status);
    } break;
    // Get status of "STOP".
    case external_command::ActionCommandType::STOP: {
      CheckScenarioFinished(special_config_.stop_scenario_name(), status);
    } break;
    // Get status of "START".
    case external_command::ActionCommandType::START: {
      auto* latest_chassis_status =
          message_reader_->GetMessage<apollo::canbus::Chassis>(
              chassis_status_name_);
      if (nullptr == latest_chassis_status) {
        status->set_status(apollo::external_command::CommandStatusType::ERROR);
        status->set_message("Cannot get chassis status!");
        return true;
      }
      // If the vehicle is moving, the command is considered as finished.
      if (latest_chassis_status->speed_mps() >
          special_config_.minimun_non_zero_speed()) {
        status->set_status(
            apollo::external_command::CommandStatusType::FINISHED);
      }
      status->set_status(apollo::external_command::CommandStatusType::RUNNING);
    } break;
    // Get status of "CLEAR_PLANNING".
    case external_command::ActionCommandType::CLEAR_PLANNING: {
      status->set_status(apollo::external_command::CommandStatusType::FINISHED);
    } break;
    // Get status of "SWITCH_TO_MANUAL".
    case external_command::ActionCommandType::SWITCH_TO_MANUAL: {
      CheckModeSwitchFinished(canbus::Chassis::COMPLETE_MANUAL, status);
    } break;
    // Get status of "SWITCH_TO_AUTO".
    case external_command::ActionCommandType::SWITCH_TO_AUTO: {
      CheckModeSwitchFinished(canbus::Chassis::COMPLETE_AUTO_DRIVE, status);
    } break;
    // Get status of "VIN_REQ".
    default: {
      // todo: Get the real result of control.
      status->set_status(apollo::external_command::CommandStatusType::FINISHED);
    } break;
  }
  return true;
}

void ActionCommandProcessor::OnCommand(
    const std::shared_ptr<ActionCommand>& command,
    std::shared_ptr<CommandStatus>& status) {
  std::string module_name = "UNKNOWN";
  if (command->has_header()) {
    module_name = command->header().module_name();
    double timestamp = apollo::cyber::Clock::NowInSeconds();
    AINFO << std::setprecision(12) << "timestamp: " << timestamp << " "
          << "request for " << command->header().timestamp_sec();
    if (timestamp - command->header().timestamp_sec() > 2.0) {
      AINFO << "request for " << command->header().module_name()
            << " has been timeouted";
      return;
    }
  }
  status->set_status(CommandStatusType::RUNNING);
  status->set_command_id(command->command_id());
  switch (command->command()) {
      // Send "FOLLOW" message to planning.
    case external_command::ActionCommandType::FOLLOW: {
      planning::PadMessage planning_message;
      common::util::FillHeader(module_name, &planning_message);
      planning_message.set_action(planning::PadMessage::FOLLOW);
      planning_action_writer_->Write(planning_message);
    } break;
    // Send "CHANGE_LEFT" message to planning.
    case external_command::ActionCommandType::CHANGE_LEFT: {
      planning::PadMessage planning_message;
      common::util::FillHeader(module_name, &planning_message);
      planning_message.set_action(planning::PadMessage::CHANGE_LEFT);
      planning_action_writer_->Write(planning_message);
    } break;
    // Send "CHANGE_RIGHT" message to planning.
    case external_command::ActionCommandType::CHANGE_RIGHT: {
      planning::PadMessage planning_message;
      common::util::FillHeader(module_name, &planning_message);
      planning_message.set_action(planning::PadMessage::CHANGE_RIGHT);
      planning_action_writer_->Write(planning_message);
    } break;
    // Send "PULL_OVER" message to planning.
    case external_command::ActionCommandType::PULL_OVER: {
      planning::PadMessage planning_message;
      common::util::FillHeader(module_name, &planning_message);
      planning_message.set_action(planning::PadMessage::PULL_OVER);
      planning_action_writer_->Write(planning_message);
    } break;
    // Send "STOP" message to planning.
    case external_command::ActionCommandType::STOP: {
      planning::PadMessage planning_message;
      common::util::FillHeader(module_name, &planning_message);
      planning_message.set_action(planning::PadMessage::STOP);
      planning_action_writer_->Write(planning_message);
    } break;
    // Send "CLEAR_PLANNING" message to planning.
    case external_command::ActionCommandType::CLEAR_PLANNING: {
      planning::PadMessage planning_message;
      common::util::FillHeader(module_name, &planning_message);
      planning_message.set_action(planning::PadMessage::CLEAR_PLANNING);
      planning_action_writer_->Write(planning_message);
      status->set_status(CommandStatusType::FINISHED);
    } break;
    // Send "START" message to planning.
    case external_command::ActionCommandType::START: {
      planning::PadMessage planning_message;
      common::util::FillHeader(module_name, &planning_message);
      planning_message.set_action(planning::PadMessage::RESUME_CRUISE);
      planning_action_writer_->Write(planning_message);
    } break;
    // Send "SWITCH_TO_MANUAL" message to control.
    case external_command::ActionCommandType::SWITCH_TO_MANUAL: {
      // Use async function to wait for the chassis to be in manual mode.
      cyber::Async(&ActionCommandProcessor::SwitchToManualMode, this,
                   module_name);
    } break;
    // Send "SWITCH_TO_AUTO" message to control.
    case external_command::ActionCommandType::SWITCH_TO_AUTO: {
      // Chassis need be switched to manual mode before switch to auto mode.
      // Use async function to wait for the chassis to be in auto mode.
      cyber::Async(&ActionCommandProcessor::SwitchToAutoMode, this,
                   module_name);
    } break;
    // Send "ENTER_MISSION" message to planning.
    case external_command::ActionCommandType::ENTER_MISSION: {
      planning::PadMessage planning_message;
      common::util::FillHeader(module_name, &planning_message);
      planning_message.set_action(planning::PadMessage::ENTER_MISSION);
      planning_action_writer_->Write(planning_message);
    } break;
    // Send "EXIT_MISSION" message to planning.
    case external_command::ActionCommandType::EXIT_MISSION: {
      planning::PadMessage planning_message;
      common::util::FillHeader(module_name, &planning_message);
      planning_message.set_action(planning::PadMessage::EXIT_MISSION);
      planning_action_writer_->Write(planning_message);
    } break;
    // Send "VIN_REQ" message to control.
    default: {
      control::PadMessage control_message;
      common::util::FillHeader(module_name, &control_message);
      control_message.set_action(control::DrivingAction::VIN_REQ);
      control_action_writer_->Write(control_message);
    } break;
  }
}

void ActionCommandProcessor::SwitchToAutoMode(const std::string& module_name) {
  last_mode_switch_status_.set_status(
      apollo::external_command::CommandStatusType::RUNNING);
  // Get the chassis' driving mode first.
  auto* latest_chassis_status =
      message_reader_->GetMessage<apollo::canbus::Chassis>(
          chassis_status_name_);
  if (nullptr == latest_chassis_status) {
    UpdateModeSwitchStatus("Failed to get chassis data!", true);
    return;
  }
  // Do nothing if chassis is already in auto mode.
  if (canbus::Chassis::COMPLETE_AUTO_DRIVE ==
      latest_chassis_status->driving_mode()) {
    UpdateModeSwitchStatus("Chassis is already in auto mode.", false);
    return;
  }
  // Make sure chassis is in manual mode.
  if (!SwitchMode(canbus::Chassis::COMPLETE_MANUAL,
                  control::DrivingAction::RESET, module_name)) {
    UpdateModeSwitchStatus("Chassis cannot be switched to manual mode first!",
                           true);
    return;
  }
  // Switch to auto mode.
  SwitchMode(canbus::Chassis::COMPLETE_AUTO_DRIVE,
             control::DrivingAction::START, module_name);
}

void ActionCommandProcessor::SwitchToManualMode(
    const std::string& module_name) {
  last_mode_switch_status_.set_status(
      apollo::external_command::CommandStatusType::RUNNING);
  SwitchMode(canbus::Chassis::COMPLETE_MANUAL, control::DrivingAction::RESET,
             module_name);
}

bool ActionCommandProcessor::SwitchMode(
    const canbus::Chassis::DrivingMode& driving_mode,
    const control::DrivingAction& driving_action,
    const std::string& module_name) {
  const std::string& mode_name =
      canbus::Chassis::DrivingMode_Name(driving_mode);
  // Get the chassis' driving mode first.
  auto* latest_chassis_status =
      message_reader_->GetMessage<apollo::canbus::Chassis>(
          chassis_status_name_);
  if (nullptr == latest_chassis_status) {
    UpdateModeSwitchStatus("Failed to get chassis data!", true);
    return false;
  }
  // Do nothing if chassis is already in manual mode.
  if (driving_mode == latest_chassis_status->driving_mode()) {
    UpdateModeSwitchStatus("Chassis is already in " + mode_name, false);
    return true;
  }
  control::PadMessage control_message;
  common::util::FillHeader(module_name, &control_message);
  static constexpr int kMaxTries = 3;
  static constexpr auto kTryInterval = std::chrono::milliseconds(500);
  control_message.set_action(driving_action);
  bool is_switch_success = false;
  for (int i = 0; i < kMaxTries; ++i) {
    // Send driving action periodically until entering target driving mode.
    control_action_writer_->Write(control_message);
    std::this_thread::sleep_for(kTryInterval);

    latest_chassis_status =
        message_reader_->GetMessage<apollo::canbus::Chassis>(
            chassis_status_name_);
    if (nullptr == latest_chassis_status) {
      UpdateModeSwitchStatus("Failed to get chassis data!", true);
      return false;
    }
    if (latest_chassis_status->driving_mode() == driving_mode) {
      is_switch_success = true;
      break;
    }
  }
  if (!is_switch_success) {
    UpdateModeSwitchStatus("Chassis cannot be switched to " + mode_name, true);
    return false;
  }
  return true;
}

void ActionCommandProcessor::CheckScenarioFinished(
    const std::string& scenario_name, CommandStatus* status) const {
  // When planning is in the given scenario and the scenario is finished,
  // command status is finished.
  status->set_status(apollo::external_command::CommandStatusType::UNKNOWN);
  status->set_message("Cannot get planning scenario status!");
  auto* latest_planning_status =
      message_reader_->GetMessage<apollo::planning::ADCTrajectory>(
          planning_status_name_);
  if (nullptr == latest_planning_status) {
    status->set_status(apollo::external_command::CommandStatusType::ERROR);
    status->set_message("Cannot get planning status!");
    return;
  }
  if (!latest_planning_status->has_debug() ||
      latest_planning_status->debug().has_planning_data()) {
    return;
  }
  const auto& debug = latest_planning_status->debug().planning_data();
  if (!debug.has_scenario()) {
    return;
  }
  const auto& scenario = debug.scenario();
  if (!scenario.has_scenario_plugin_type()) {
    return;
  }
  auto* latest_planning_command_status =
      message_reader_->GetMessage<CommandStatus>(planning_command_status_name_);
  if (nullptr == latest_planning_command_status) {
    status->set_status(apollo::external_command::CommandStatusType::ERROR);
    status->set_message("Cannot get planning command status!");
    return;
  }
  status->set_message("");
  const auto& scenario_type = scenario.scenario_plugin_type();
  if (scenario_type == scenario_name &&
      latest_planning_command_status->status() ==
          apollo::external_command::CommandStatusType::FINISHED) {
    status->set_status(apollo::external_command::CommandStatusType::FINISHED);
    return;
  }
  status->set_status(apollo::external_command::CommandStatusType::RUNNING);
}

void ActionCommandProcessor::CheckModeSwitchFinished(
    const apollo::canbus::Chassis::DrivingMode& drive_mode,
    CommandStatus* status) const {
  auto* latest_chassis_status =
      message_reader_->GetMessage<apollo::canbus::Chassis>(
          chassis_status_name_);
  if (nullptr == latest_chassis_status) {
    status->set_status(apollo::external_command::CommandStatusType::ERROR);
    status->set_message("Cannot get chassis status!");
    return;
  }
  if (latest_chassis_status->has_driving_mode() &&
      latest_chassis_status->driving_mode() == drive_mode) {
    status->set_status(apollo::external_command::CommandStatusType::FINISHED);
    return;
  }
  status->CopyFrom(last_mode_switch_status_);
}

void ActionCommandProcessor::UpdateModeSwitchStatus(const std::string& message,
                                                    bool is_error) {
  if (is_error) {
    last_mode_switch_status_.set_status(
        apollo::external_command::CommandStatusType::ERROR);
  } else {
    last_mode_switch_status_.set_status(
        apollo::external_command::CommandStatusType::FINISHED);
  }
  last_mode_switch_status_.set_message(message);
  AERROR << message;
}

}  // namespace external_command
}  // namespace apollo
