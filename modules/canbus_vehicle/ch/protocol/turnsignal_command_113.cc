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

#include "modules/canbus_vehicle/ch/protocol/turnsignal_command_113.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

const int32_t Turnsignalcommand113::ID = 0x113;

// public
Turnsignalcommand113::Turnsignalcommand113() { Reset(); }

uint32_t Turnsignalcommand113::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Turnsignalcommand113::UpdateData(uint8_t* data) {
  set_p_turn_signal_cmd(data, turn_signal_cmd_);
  set_p_low_beam_cmd(data, low_beam_cmd_);
}

void Turnsignalcommand113::Reset() {
  // TODO(All) :  you should check this manually
  turn_signal_cmd_ = Turnsignal_command_113::TURN_SIGNAL_CMD_NONE;
  low_beam_cmd_ = Turnsignal_command_113::LOW_BEAM_CMD_OFF;
}

Turnsignalcommand113* Turnsignalcommand113::set_turn_signal_cmd(
    Turnsignal_command_113::Turn_signal_cmdType turn_signal_cmd) {
  turn_signal_cmd_ = turn_signal_cmd;
  return this;
}

// config detail: {'bit': 0, 'description': 'Lighting control(Command)', 'enum':
// {0: 'TURN_SIGNAL_CMD_NONE', 1: 'TURN_SIGNAL_CMD_LEFT', 2:
// 'TURN_SIGNAL_CMD_RIGHT', 3: 'TURN_SIGNAL_CMD_HAZARD_WARNING_LAMPSTS'},
// 'is_signed_var': False, 'len': 8, 'name': 'TURN_SIGNAL_CMD', 'offset': 0.0,
// 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'enum'}
void Turnsignalcommand113::set_p_turn_signal_cmd(
    uint8_t* data,
    Turnsignal_command_113::Turn_signal_cmdType turn_signal_cmd) {
  int x = turn_signal_cmd;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

Turnsignalcommand113* Turnsignalcommand113::set_low_beam_cmd(
    Turnsignal_command_113::Low_beam_cmdType low_beam_cmd) {
  low_beam_cmd_ = low_beam_cmd;
  return this;
}

// config detail: {'bit': 8, 'description': 'Lighting control(Command)', 'enum':
// {0: 'LOW_BEAM_CMD_OFF', 1: 'LOW_BEAM_CMD_ON'}, 'is_signed_var': False, 'len':
// 2, 'name': 'LOW_BEAM_CMD', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Turnsignalcommand113::set_p_low_beam_cmd(
    uint8_t* data, Turnsignal_command_113::Low_beam_cmdType low_beam_cmd) {
  int x = low_beam_cmd;

  Byte to_set(data + 1);
  to_set.set_value(static_cast<uint8_t>(x), 0, 2);
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
