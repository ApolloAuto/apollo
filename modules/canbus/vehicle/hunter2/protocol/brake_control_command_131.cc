/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/hunter2/protocol/brake_control_command_131.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

using ::apollo::drivers::canbus::Byte;

const int32_t Brakecontrolcommand131::ID = 0x131;

// public
Brakecontrolcommand131::Brakecontrolcommand131() { Reset(); }

uint32_t Brakecontrolcommand131::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Brakecontrolcommand131::UpdateData(uint8_t* data) {
  set_p_brake_control_command(data, brake_control_command_);
}

void Brakecontrolcommand131::Reset() {
  // TODO(All) :  you should check this manually
  brake_control_command_ = Brake_control_command_131::BRAKE_CONTROL_COMMAND_BRAKE_RELEASE;
}

Brakecontrolcommand131* Brakecontrolcommand131::set_brake_control_command(
    Brake_control_command_131::Brake_control_commandType brake_control_command) {
  brake_control_command_ = brake_control_command;
  return this;
 }

// config detail: {'bit': 7, 'enum': {0: 'BRAKE_CONTROL_COMMAND_BRAKE_RELEASE', 1: 'BRAKE_CONTROL_COMMAND_BRAKE_LOCK'}, 'is_signed_var': False, 'len': 8, 'name': 'brake_control_command', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Brakecontrolcommand131::set_p_brake_control_command(uint8_t* data,
    Brake_control_command_131::Brake_control_commandType brake_control_command) {
  int x = brake_control_command;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}

}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo
