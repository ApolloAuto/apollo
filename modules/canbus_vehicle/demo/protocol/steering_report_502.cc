/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/demo/protocol/steering_report_502.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace demo {

using ::apollo::drivers::canbus::Byte;

Steeringreport502::Steeringreport502() {}
const int32_t Steeringreport502::ID = 0x502;

void Steeringreport502::Parse(const std::uint8_t* bytes, int32_t length,
                              Demo* chassis) const {
  chassis->mutable_steering_report_502()->set_steer_angle_rear_actual(
      steer_angle_rear_actual(bytes, length));
  chassis->mutable_steering_report_502()->set_steer_angle_spd_actual(
      steer_angle_spd_actual(bytes, length));
  chassis->mutable_steering_report_502()->set_steer_flt2(
      steer_flt2(bytes, length));
  chassis->mutable_steering_report_502()->set_steer_flt1(
      steer_flt1(bytes, length));
  chassis->mutable_steering_report_502()->set_steer_en_state(
      steer_en_state(bytes, length));
  chassis->mutable_steering_report_502()->set_steer_angle_actual(
      steer_angle_actual(bytes, length));
}

// config detail: {'bit': 47, 'is_signed_var': False, 'len': 16, 'name':
// 'steer_angle_rear_actual', 'offset': -500.0, 'order': 'motorola',
// 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0,
// 'type': 'int'}
int Steeringreport502::steer_angle_rear_actual(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x + -500.000000;
  return ret;
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
// 'steer_angle_spd_actual', 'offset': 0.0, 'order': 'motorola',
// 'physical_range': '[0|0]', 'physical_unit': 'deg/s', 'precision': 1.0,
// 'type': 'int'}
int Steeringreport502::steer_angle_spd_actual(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 23, 'description': 'Steer system communication fault',
// 'enum': {0: 'STEER_FLT2_NO_FAULT', 1:
// 'STEER_FLT2_STEER_SYSTEM_COMUNICATION_FAULT'}, 'is_signed_var': False, 'len':
// 8, 'name': 'steer_flt2', 'offset': 0.0, 'order': 'motorola',
// 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Steering_report_502::Steer_flt2Type Steeringreport502::steer_flt2(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Steering_report_502::Steer_flt2Type ret =
      static_cast<Steering_report_502::Steer_flt2Type>(x);
  return ret;
}

// config detail: {'bit': 15, 'description': 'Steer system hardware fault',
// 'enum': {0: 'STEER_FLT1_NO_FAULT', 1:
// 'STEER_FLT1_STEER_SYSTEM_HARDWARE_FAULT'}, 'is_signed_var': False, 'len': 8,
// 'name': 'steer_flt1', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Steering_report_502::Steer_flt1Type Steeringreport502::steer_flt1(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Steering_report_502::Steer_flt1Type ret =
      static_cast<Steering_report_502::Steer_flt1Type>(x);
  return ret;
}

// config detail: {'bit': 1, 'description': 'enable', 'enum': {0:
// 'STEER_EN_STATE_MANUAL', 1: 'STEER_EN_STATE_AUTO', 2:
// 'STEER_EN_STATE_TAKEOVER', 3: 'STEER_EN_STATE_STANDBY'}, 'is_signed_var':
// False, 'len': 2, 'name': 'steer_en_state', 'offset': 0.0, 'order':
// 'motorola', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0,
// 'signal_type': 'enable', 'type': 'enum'}
Steering_report_502::Steer_en_stateType Steeringreport502::steer_en_state(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  Steering_report_502::Steer_en_stateType ret =
      static_cast<Steering_report_502::Steer_en_stateType>(x);
  return ret;
}

// config detail: {'bit': 31, 'description': 'command', 'is_signed_var': False,
// 'len': 16, 'name': 'steer_angle_actual', 'offset': -500.0, 'order':
// 'motorola', 'physical_range': '[-500|500]', 'physical_unit': 'deg',
// 'precision': 1.0, 'signal_type': 'command', 'type': 'int'}
int Steeringreport502::steer_angle_actual(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x + -500.000000;
  return ret;
}
}  // namespace demo
}  // namespace canbus
}  // namespace apollo
