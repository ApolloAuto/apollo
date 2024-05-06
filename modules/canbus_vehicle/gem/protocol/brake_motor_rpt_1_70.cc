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

#include "modules/canbus_vehicle/gem/protocol/brake_motor_rpt_1_70.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Brakemotorrpt170::Brakemotorrpt170() {}
const int32_t Brakemotorrpt170::ID = 0x70;

void Brakemotorrpt170::Parse(const std::uint8_t* bytes, int32_t length,
                             Gem* chassis) const {
  chassis->mutable_brake_motor_rpt_1_70()->set_motor_current(
      motor_current(bytes, length));
  chassis->mutable_brake_motor_rpt_1_70()->set_shaft_position(
      shaft_position(bytes, length));
}

// config detail: {'name': 'motor_current', 'offset': 0.0, 'precision': 0.001,
// 'len': 32, 'is_signed_var': False, 'physical_range': '[0|4294967.295]',
// 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'amps'}
double Brakemotorrpt170::motor_current(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 2);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 3);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'shaft_position', 'offset': 0.0, 'precision': 0.001,
// 'len': 32, 'is_signed_var': True, 'physical_range':
// '[-2147483.648|2147483.647]', 'bit': 39, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'radians'}
double Brakemotorrpt170::shaft_position(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 6);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 7);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 0;
  x >>= 0;

  double ret = x * 0.001000;
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
