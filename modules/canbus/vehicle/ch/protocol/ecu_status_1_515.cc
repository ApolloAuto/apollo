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

#include "modules/canbus/vehicle/ch/protocol/ecu_status_1_515.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Ecustatus1515::Ecustatus1515() {}
const int32_t Ecustatus1515::ID = 0x515;

void Ecustatus1515::Parse(const std::uint8_t* bytes, int32_t length,
                          ChassisDetail* chassis) const {
  chassis->mutable_ch()->mutable_ecu_status_1_515()->set_speed(
      speed(bytes, length));
  chassis->mutable_ch()->mutable_ecu_status_1_515()->set_acc_speed(
      acc_speed(bytes, length));
  chassis->mutable_ch()->mutable_ecu_status_1_515()->set_ctrl_sts(
      ctrl_sts(bytes, length));
  chassis->mutable_ch()->mutable_ecu_status_1_515()->set_chassis_sts(
      chassis_sts(bytes, length));
  chassis->mutable_ch()->mutable_ecu_status_1_515()->set_chassis_err(
      chassis_err(bytes, length));
}

// config detail: {'description': 'Current speed (Steering status)', 'offset':
// 0.0, 'precision': 0.01, 'len': 16, 'name': 'speed', 'is_signed_var': True,
// 'physical_range': '[0|0]', 'bit': 0, 'type': 'double', 'order': 'intel',
// 'physical_unit': 'm/s'}
double Ecustatus1515::speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'description': 'Current acceleration (Steering status)',
// 'offset': 0.0, 'precision': 0.001, 'len': 16, 'name': 'acc_speed',
// 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 16, 'type':
// 'double', 'order': 'intel', 'physical_unit': 'm/s^2'}
double Ecustatus1515::acc_speed(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'description': 'Current Auto-mode state (Chassis status)',
// 'enum': {0: 'CTRL_STS_OUT_OF_CONTROL', 1: 'CTRL_STS_UNDER_CONTROL'},
// 'precision': 1.0, 'len': 8, 'name': 'ctrl_sts', 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order':
// 'intel', 'physical_unit': ''}
Ecu_status_1_515::Ctrl_stsType Ecustatus1515::ctrl_sts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Ecu_status_1_515::Ctrl_stsType ret =
      static_cast<Ecu_status_1_515::Ctrl_stsType>(x);
  return ret;
}

// config detail: {'description': 'Current chassis state (Chassis status)',
// 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name': 'chassis_sts',
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 40, 'type':
// 'int', 'order': 'intel', 'physical_unit': ''}
int Ecustatus1515::chassis_sts(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Chassis error code (Chassis status)',
// 'offset': 0.0, 'precision': 1.0, 'len': 16, 'name': 'chassis_err',
// 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 48, 'type':
// 'int', 'order': 'intel', 'physical_unit': ''}
int Ecustatus1515::chassis_err(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
