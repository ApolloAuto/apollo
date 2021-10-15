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

#include "modules/canbus/vehicle/hunter2/protocol/motion_control_instruction_111.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

using ::apollo::drivers::canbus::Byte;

const int32_t Motioncontrolinstruction111::ID = 0x111;

// public
Motioncontrolinstruction111::Motioncontrolinstruction111() { Reset(); }

uint32_t Motioncontrolinstruction111::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Motioncontrolinstruction111::UpdateData(uint8_t* data) {
  set_p_steer_instruction(data, steer_instruction_);
  set_p_speed_instruction(data, speed_instruction_);
}

void Motioncontrolinstruction111::Reset() {
  // TODO(All) :  you should check this manually
  steer_instruction_ = 0.0;
  speed_instruction_ = 0.0;
}

Motioncontrolinstruction111* Motioncontrolinstruction111::set_steer_instruction(
    double steer_instruction) {
  steer_instruction_ = steer_instruction;
  return this;
 }

// config detail: {'bit': 55, 'is_signed_var': True, 'len': 16, 'name': 'steer_instruction', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'rad', 'precision': 0.001, 'type': 'double'}
void Motioncontrolinstruction111::set_p_steer_instruction(uint8_t* data,
    double steer_instruction) {
  steer_instruction = ProtocolData::BoundedValue(-32.768, 32.767, steer_instruction);
  int x = steer_instruction / 0.001000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 7);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 6);
  to_set1.set_value(t, 0, 8);
}


Motioncontrolinstruction111* Motioncontrolinstruction111::set_speed_instruction(
    double speed_instruction) {
  speed_instruction_ = speed_instruction;
  return this;
 }

// config detail: {'bit': 7, 'is_signed_var': True, 'len': 16, 'name': 'speed_instruction', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
void Motioncontrolinstruction111::set_p_speed_instruction(uint8_t* data,
    double speed_instruction) {
  speed_instruction = ProtocolData::BoundedValue(-32.768, 32.767, speed_instruction);
  int x = speed_instruction / 0.001000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 1);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 0);
  to_set1.set_value(t, 0, 8);
}

}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo
