/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/gem/protocol/turn_cmd_63.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

const int32_t Turncmd63::ID = 0x63;

// public
Turncmd63::Turncmd63() { Reset(); }

uint32_t Turncmd63::GetPeriod() const {
  // TODO(QiL) :modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Turncmd63::UpdateData(uint8_t* data) {
  set_p_turn_signal_cmd(data, turn_signal_cmd_);
}

void Turncmd63::Reset() {
  // TODO(QiL) :you should check this manually
  turn_signal_cmd_ = Turn_cmd_63::TURN_SIGNAL_CMD_NONE;
}

Turncmd63* Turncmd63::set_turn_signal_cmd(
    Turn_cmd_63::Turn_signal_cmdType turn_signal_cmd) {
  turn_signal_cmd_ = turn_signal_cmd;
  return this;
}

// config detail: {'name': 'TURN_SIGNAL_CMD', 'enum': {0:
// 'TURN_SIGNAL_CMD_RIGHT', 1: 'TURN_SIGNAL_CMD_NONE', 2:
// 'TURN_SIGNAL_CMD_LEFT', 3: 'TURN_SIGNAL_CMD_HAZARD'}, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]',
// 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Turncmd63::set_p_turn_signal_cmd(
    uint8_t* data, Turn_cmd_63::Turn_signal_cmdType turn_signal_cmd) {
  uint8_t x = turn_signal_cmd;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
