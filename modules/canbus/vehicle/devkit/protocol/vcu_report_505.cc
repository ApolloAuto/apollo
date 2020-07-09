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

#include "modules/canbus/vehicle/devkit/protocol/vcu_report_505.h"

#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::drivers::canbus::Byte;

Vcureport505::Vcureport505() {}
const int32_t Vcureport505::ID = 0x505;

void Vcureport505::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_devkit()->mutable_vcu_report_505()->set_acc(
      acc(bytes, length));
  chassis->mutable_devkit()->mutable_vcu_report_505()->set_speed(
      speed(bytes, length));
}

// config detail: {'name': 'acc', 'offset': 0.0, 'precision': 0.01, 'len': 12,
// 'is_signed_var': True, 'physical_range': '[-10|10]', 'bit': 7, 'type':
// 'double', 'order': 'motorola', 'physical_unit': ''}
double Vcureport505::acc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'speed', 'offset': 0.0, 'precision': 0.001, 'len':
// 16, 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit': 23,
// 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Vcureport505::speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}
}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
