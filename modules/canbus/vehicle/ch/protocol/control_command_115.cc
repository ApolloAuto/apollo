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

#include "modules/canbus/vehicle/ch/protocol/control_command_115.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

const int32_t Controlcommand115::ID = 0x115;

// public
Controlcommand115::Controlcommand115() { Reset(); }

uint32_t Controlcommand115::GetPeriod() const {
  // modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Controlcommand115::UpdateData(uint8_t* data) {
  set_p_ctrl_cmd(data, ctrl_cmd_);
}

void Controlcommand115::Reset() {
  // you should check this manually
  ctrl_cmd_ = Control_command_115::CTRL_CMD_OUT_OF_CONTROL;
}

Controlcommand115* Controlcommand115::set_ctrl_cmd(
    Control_command_115::Ctrl_cmdType ctrl_cmd) {
  ctrl_cmd_ = ctrl_cmd;
  return this;
}

// config detail: {'description': 'Take control(Command)', 'enum': {0:
// 'CTRL_CMD_OUT_OF_CONTROL', 1: 'CTRL_CMD_UNDER_CONTROL'}, 'precision': 1.0,
// 'len': 8, 'name': 'CTRL_CMD', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'intel',
// 'physical_unit': ''}
void Controlcommand115::set_p_ctrl_cmd(
    uint8_t* data, Control_command_115::Ctrl_cmdType ctrl_cmd) {
  int x = ctrl_cmd;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
