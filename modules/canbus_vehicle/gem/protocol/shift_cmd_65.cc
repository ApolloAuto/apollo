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

#include "modules/canbus/vehicle/gem/protocol/shift_cmd_65.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

const int32_t Shiftcmd65::ID = 0x65;

// public
Shiftcmd65::Shiftcmd65() { Reset(); }

uint32_t Shiftcmd65::GetPeriod() const {
  // TODO(QiL) :modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Shiftcmd65::UpdateData(uint8_t* data) {
  set_p_shift_cmd(data, shift_cmd_);
}

void Shiftcmd65::Reset() {
  // TODO(QiL) :you should check this manually
  shift_cmd_ = Shift_cmd_65::SHIFT_CMD_PARK;
}

Shiftcmd65* Shiftcmd65::set_shift_cmd(Shift_cmd_65::Shift_cmdType shift_cmd) {
  shift_cmd_ = shift_cmd;
  return this;
}

// config detail: {'description':
// 'FORWARD_is_also_LOW_on_vehicles_with_LOW/HIGH,_PARK_and_HIGH_only_available_on_certain_Vehicles',
// 'enum': {0: 'SHIFT_CMD_PARK', 1: 'SHIFT_CMD_REVERSE', 2: 'SHIFT_CMD_NEUTRAL',
// 3: 'SHIFT_CMD_FORWARD', 4: 'SHIFT_CMD_LOW'}, 'precision': 1.0, 'len': 8,
// 'name': 'SHIFT_CMD', 'is_signed_var': False, 'offset': 0.0, 'physical_range':
// '[0|4]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Shiftcmd65::set_p_shift_cmd(uint8_t* data,
                                 Shift_cmd_65::Shift_cmdType shift_cmd) {
  uint8_t x = shift_cmd;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
