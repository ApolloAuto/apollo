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
 * @file chassis_command_processor.cc
 **/

#include "modules/external_command/command_processor/chassis_command_processor/chassis_command_processor.h"

#include "modules/external_command/command_processor/command_processor_base/proto/command_processor_config.pb.h"

#include "modules/external_command/command_processor/command_processor_base/util/message_reader.h"
#include "modules/external_command/command_processor/command_processor_base/util/message_writer.h"

namespace apollo {
namespace external_command {

bool ChassisCommandProcessor::Init(const std::shared_ptr<cyber::Node>& node) {
    // hook: Apollo License Verification: v_apollo_park
    if (!CommandProcessorBase::Init(node)) {
        return false;
    }
    last_received_command_.set_command_id(-1);
    message_reader_ = MessageReader::Instance();
    const auto& config = GetProcessorConfig();
    command_service_ = node->CreateService<ChassisCommand, CommandStatus>(
            config.input_command_name(),
            [this](const std::shared_ptr<ChassisCommand>& command, std::shared_ptr<CommandStatus>& status) {
                this->OnCommand(command, status);
            });
    // Create writer for output command.
    CHECK(config.output_command_name().size() > 0 && config.input_command_status_name().size() > 0);
    auto message_writer = MessageWriter::Instance();
    chassis_command_writer_ = message_writer->RegisterMessage<ChassisCommand>(config.output_command_name().Get(0));
    chassis_status_name_ = config.input_command_status_name().Get(0);
    message_reader_->RegisterMessage<apollo::canbus::Chassis>(chassis_status_name_);
    return true;
}

// command = request.command_id status = response.get();
bool ChassisCommandProcessor::GetCommandStatus(int64_t command_id, CommandStatus* status) const {
    CHECK_NOTNULL(status);
    if (command_id != last_received_command_.command_id()) {
        return false;
    }
    status->set_command_id(command_id);
    auto* latest_chassis_status = message_reader_->GetMessage<apollo::canbus::Chassis>(chassis_status_name_);
    if (nullptr == latest_chassis_status) {
        status->set_status(apollo::external_command::CommandStatusType::ERROR);
        status->set_message("Cannot get chassis status!");
        return true;
    }
    // Process basic vehicle signals.
    if (last_received_command_.has_basic_signal()) {
        const auto& basic_signal = last_received_command_.basic_signal();
        // Check turn signal.
        if (basic_signal.has_turn_signal()) {
            if (!latest_chassis_status->has_signal() || !latest_chassis_status->signal().has_turn_signal()
                || latest_chassis_status->signal().turn_signal() != basic_signal.turn_signal()) {
                status->set_status(apollo::external_command::CommandStatusType::RUNNING);
                return true;
            }
        }
        // Check high beam.
        if (basic_signal.has_high_beam()) {
            if (!latest_chassis_status->has_signal() || !latest_chassis_status->signal().has_high_beam()
                || latest_chassis_status->signal().high_beam() != basic_signal.high_beam()) {
                status->set_status(apollo::external_command::CommandStatusType::RUNNING);
                return true;
            }
        }
        // Check low beam.
        if (basic_signal.has_low_beam()) {
            if (!latest_chassis_status->has_signal() || !latest_chassis_status->signal().has_low_beam()
                || latest_chassis_status->signal().low_beam() != basic_signal.low_beam()) {
                status->set_status(apollo::external_command::CommandStatusType::RUNNING);
                return true;
            }
        }
        // Check horn.
        if (basic_signal.has_horn()) {
            if (!latest_chassis_status->has_signal() || !latest_chassis_status->signal().has_horn()
                || latest_chassis_status->signal().horn() != basic_signal.horn()) {
                status->set_status(apollo::external_command::CommandStatusType::RUNNING);
                return true;
            }
        }
        // Check emergency light.
        if (basic_signal.has_emergency_light()) {
            if (!latest_chassis_status->has_signal() || !latest_chassis_status->signal().has_emergency_light()
                || latest_chassis_status->signal().emergency_light() != basic_signal.emergency_light()) {
                status->set_status(apollo::external_command::CommandStatusType::RUNNING);
                return true;
            }
        }
    }
    status->set_status(apollo::external_command::CommandStatusType::FINISHED);
    return true;
}

void ChassisCommandProcessor::OnCommand(
        const std::shared_ptr<ChassisCommand>& command,
        std::shared_ptr<CommandStatus>& status) {
    if (command->has_header()) {
        double timestamp = apollo::cyber::Clock::NowInSeconds();
        AINFO << std::setprecision(12) << "timestamp: " << timestamp << " " << "request for "
              << command->header().timestamp_sec();
        if (timestamp - command->header().timestamp_sec() > 2.0) {
            AINFO << "request for " << command->header().module_name() << " has been timeouted";
            return;
        }
    }
    chassis_command_writer_->Write(command);
    status->set_command_id(command->command_id());
    status->set_status(apollo::external_command::CommandStatusType::RUNNING);
    last_received_command_.CopyFrom(*command);
}

}  // namespace external_command
}  // namespace apollo
