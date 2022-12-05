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

#include "modules/canbus_vehicle/ch/protocol/steer_command_112.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

const int32_t Steercommand112::ID = 0x112;

// public
Steercommand112::Steercommand112() { Reset(); }

uint32_t Steercommand112::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Steercommand112::UpdateData(uint8_t* data) {
  set_p_steer_angle_en_ctrl(data, steer_angle_en_ctrl_);
  set_p_steer_angle_cmd(data, steer_angle_cmd_);
}

void Steercommand112::Reset() {
  // TODO(All) :  you should check this manually
  steer_angle_en_ctrl_ = Steer_command_112::STEER_ANGLE_EN_CTRL_DISABLE;
  steer_angle_cmd_ = 0.0;
}

Steercommand112* Steercommand112::set_steer_angle_en_ctrl(
    Steer_command_112::Steer_angle_en_ctrlType steer_angle_en_ctrl) {
  steer_angle_en_ctrl_ = steer_angle_en_ctrl;
  return this;
}

// config detail: {'bit': 0, 'description': 'steering angle enable
// bit(Command)', 'enum': {0: 'STEER_ANGLE_EN_CTRL_DISABLE', 1:
// 'STEER_ANGLE_EN_CTRL_ENABLE'}, 'is_signed_var': False, 'len': 8, 'name':
// 'STEER_ANGLE_EN_CTRL', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Steercommand112::set_p_steer_angle_en_ctrl(
    uint8_t* data,
    Steer_command_112::Steer_angle_en_ctrlType steer_angle_en_ctrl) {
  int x = steer_angle_en_ctrl;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

Steercommand112* Steercommand112::set_steer_angle_cmd(double steer_angle_cmd) {
  steer_angle_cmd_ = steer_angle_cmd;
  return this;
}

// config detail: {'bit': 8, 'description': 'Current steering angle(Command)',
// 'is_signed_var': True, 'len': 16, 'name': 'STEER_ANGLE_CMD', 'offset': 0.0,
// 'order': 'intel', 'physical_range': '[-0.524|0.524]', 'physical_unit':
// 'radian', 'precision': 0.001, 'type': 'double'}
void Steercommand112::set_p_steer_angle_cmd(uint8_t* data,
                                            double steer_angle_cmd) {
  steer_angle_cmd = ProtocolData::BoundedValue(-0.524, 0.524, steer_angle_cmd);
  int x = static_cast<int>(steer_angle_cmd / 0.001000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 1);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 2);
  to_set1.set_value(t, 0, 8);
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
