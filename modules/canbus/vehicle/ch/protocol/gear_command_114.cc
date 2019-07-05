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

#include "modules/canbus/vehicle/ch/protocol/gear_command_114.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

const int32_t Gearcommand114::ID = 0x114;

// public
Gearcommand114::Gearcommand114() { Reset(); }

uint32_t Gearcommand114::GetPeriod() const {
  // modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Gearcommand114::UpdateData(uint8_t* data) {
  set_p_gear_cmd(data, gear_cmd_);
}

void Gearcommand114::Reset() {
  // you should check this manually
  gear_cmd_ = Gear_command_114::GEAR_CMD_NEUTRAL;
}

Gearcommand114* Gearcommand114::set_gear_cmd(
    Gear_command_114::Gear_cmdType gear_cmd) {
  gear_cmd_ = gear_cmd;
  return this;
}

// config detail: {'description': 'PRND control(Command)', 'enum': {1:
// 'GEAR_CMD_PARK', 2: 'GEAR_CMD_REVERSE', 3: 'GEAR_CMD_NEUTRAL', 4:
// 'GEAR_CMD_DRIVE'}, 'precision': 1.0, 'len': 8, 'name': 'GEAR_CMD',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 0,
// 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Gearcommand114::set_p_gear_cmd(uint8_t* data,
                                    Gear_command_114::Gear_cmdType gear_cmd) {
  int x = gear_cmd;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
