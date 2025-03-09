// Copyright 2025 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-01-16
//  Author: daohu527

#include "modules/serial/serial_component.h"

#include "cyber/common/file.h"

using apollo::control::ControlCommand;

namespace apollo {
namespace serial {

SerialComponent::SerialComponent() {}

SerialComponent::~SerialComponent() { Clear(); }

bool SerialComponent::Init() {
  if (!GetProtoConfig(&serial_conf_)) {
    AERROR << "Unable to load serial conf file: " << ConfigFilePath();
    return false;
  }
  ADEBUG << "Serial conf:" << serial_conf_.ShortDebugString();

  control_ = VehicleFactory::CreateControl(serial_conf_.vehicle_type());

  cyber::ReaderConfig control_cmd_reader_config;
  control_cmd_reader_config.channel_name = FLAGS_control_command_topic;
  control_cmd_reader_config.pending_queue_size =
      FLAGS_control_cmd_pending_queue_size;

  control_command_reader_ = node_->CreateReader<ControlCommand>(
      control_cmd_reader_config,
      [this](const std::shared_ptr<ControlCommand>& cmd) {
        OnControlCommand(*cmd);
      });

  // init chassis writer
  chassis_writer_ = node_->CreateWriter<Chassis>(FLAGS_chassis_topic);

  control_->Start();
  return true;
}

void SerialComponent::Clear() { control_->Stop(); }

bool SerialComponent::Proc() {
  Chassis chassis = control_->GetChassis();
  common::util::FillHeader(node_->Name(), &chassis);
  chassis_writer_->Write(chassis);
  return true;
}

void SerialComponent::OnControlCommand(const ControlCommand& control_command) {
  control_->Send(control_command);
}

std::string SerialComponent::Name() const { return "SerialComponent"; }

}  // namespace serial
}  // namespace apollo
