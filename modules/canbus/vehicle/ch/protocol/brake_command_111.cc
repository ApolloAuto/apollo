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

#include "modules/canbus/vehicle/ch/protocol/brake_command_111.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

const int32_t Brakecommand111::ID = 0x111;

// public
Brakecommand111::Brakecommand111() { Reset(); }

uint32_t Brakecommand111::GetPeriod() const {
  // modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Brakecommand111::UpdateData(uint8_t* data) {
  set_p_brake_pedal_en_ctrl(data, brake_pedal_en_ctrl_);
  set_p_brake_pedal_cmd(data, brake_pedal_cmd_);
}

void Brakecommand111::Reset() {
  // you should check this manually
  brake_pedal_en_ctrl_ = Brake_command_111::BRAKE_PEDAL_EN_CTRL_DISABLE;
  brake_pedal_cmd_ = 0;
}

Brakecommand111* Brakecommand111::set_brake_pedal_en_ctrl(
    Brake_command_111::Brake_pedal_en_ctrlType brake_pedal_en_ctrl) {
  brake_pedal_en_ctrl_ = brake_pedal_en_ctrl;
  return this;
}

// config detail: {'description': 'brake pedal enable bit(Command)', 'enum': {0:
// 'BRAKE_PEDAL_EN_CTRL_DISABLE', 1: 'BRAKE_PEDAL_EN_CTRL_ENABLE'},
// 'precision': 1.0, 'len': 8, 'name': 'BRAKE_PEDAL_EN_CTRL', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum',
// 'order': 'intel', 'physical_unit': ''}
void Brakecommand111::set_p_brake_pedal_en_ctrl(
    uint8_t* data,
    Brake_command_111::Brake_pedal_en_ctrlType brake_pedal_en_ctrl) {
  int x = brake_pedal_en_ctrl;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

Brakecommand111* Brakecommand111::set_brake_pedal_cmd(int brake_pedal_cmd) {
  brake_pedal_cmd_ = brake_pedal_cmd;
  return this;
}

// config detail: {'description': 'Percentage of brake pedal(Command)',
// 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name': 'BRAKE_PEDAL_CMD',
// 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 8, 'type': 'int',
// 'order': 'intel', 'physical_unit': '%'}
void Brakecommand111::set_p_brake_pedal_cmd(uint8_t* data,
                                            int brake_pedal_cmd) {
  brake_pedal_cmd = ProtocolData::BoundedValue(0, 100, brake_pedal_cmd);
  int x = brake_pedal_cmd;

  Byte to_set(data + 1);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
