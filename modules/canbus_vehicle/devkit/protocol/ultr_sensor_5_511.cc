/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/devkit/protocol/ultr_sensor_5_511.h"

#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::drivers::canbus::Byte;

Ultrsensor5511::Ultrsensor5511() {}
const int32_t Ultrsensor5511::ID = 0x511;

void Ultrsensor5511::Parse(const std::uint8_t* bytes, int32_t length,
                           Devkit* chassis) const {
  chassis->mutable_ultr_sensor_5_511()->set_uiuss7_tof_direct(
      uiuss7_tof_direct(bytes, length));
  chassis->mutable_ultr_sensor_5_511()->set_uiuss6_tof_direct(
      uiuss6_tof_direct(bytes, length));
  chassis->mutable_ultr_sensor_5_511()->set_uiuss1_tof_direct(
      uiuss1_tof_direct(bytes, length));
  chassis->mutable_ultr_sensor_5_511()->set_uiuss0_tof_direct(
      uiuss0_tof_direct(bytes, length));
}

// config detail: {'name': 'uiuss7_tof_direct', 'offset': 0.0, 'precision':
// 0.01724, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]',
// 'bit': 55, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
double Ultrsensor5511::uiuss7_tof_direct(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x / 58;
  return ret;
}

// config detail: {'name': 'uiuss6_tof_direct', 'offset': 0.0, 'precision':
// 0.01724, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]',
// 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
double Ultrsensor5511::uiuss6_tof_direct(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x / 58;
  return ret;
}

// config detail: {'name': 'uiuss1_tof_direct', 'offset': 0.0, 'precision':
// 0.01724, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]',
// 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
double Ultrsensor5511::uiuss1_tof_direct(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x / 58;
  return ret;
}

// config detail: {'name': 'uiuss0_tof_direct', 'offset': 0.0, 'precision':
// 0.01724, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]',
// 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
double Ultrsensor5511::uiuss0_tof_direct(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x / 58;
  return ret;
}
}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
