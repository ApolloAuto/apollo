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

#include "modules/canbus_vehicle/gem/protocol/wheel_speed_rpt_7a.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Wheelspeedrpt7a::Wheelspeedrpt7a() {}
const int32_t Wheelspeedrpt7a::ID = 0x7A;

void Wheelspeedrpt7a::Parse(const std::uint8_t* bytes, int32_t length,
                            Gem* chassis) const {
  chassis->mutable_wheel_speed_rpt_7a()->set_wheel_spd_rear_right(
      wheel_spd_rear_right(bytes, length));
  chassis->mutable_wheel_speed_rpt_7a()->set_wheel_spd_rear_left(
      wheel_spd_rear_left(bytes, length));
  chassis->mutable_wheel_speed_rpt_7a()->set_wheel_spd_front_right(
      wheel_spd_front_right(bytes, length));
  chassis->mutable_wheel_speed_rpt_7a()->set_wheel_spd_front_left(
      wheel_spd_front_left(bytes, length));
}

// config detail: {'name': 'wheel_spd_rear_right', 'offset': 0.0,
// 'precision': 1.0, 'len': 16, 'is_signed_var': True, 'physical_range':
// '[-32768|32767]', 'bit': 55, 'type': 'int', 'order': 'motorola',
// 'physical_unit': 'rad/s'}
int Wheelspeedrpt7a::wheel_spd_rear_right(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'name': 'wheel_spd_rear_left', 'offset': 0.0,
// 'precision': 1.0, 'len': 16, 'is_signed_var': True, 'physical_range':
// '[-32768|32767]', 'bit': 39, 'type': 'int', 'order': 'motorola',
// 'physical_unit': 'rad/s'}
int Wheelspeedrpt7a::wheel_spd_rear_left(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'name': 'wheel_spd_front_right', 'offset': 0.0,
// 'precision': 1.0, 'len': 16, 'is_signed_var': True, 'physical_range':
// '[-32768|32767]', 'bit': 23, 'type': 'int', 'order': 'motorola',
// 'physical_unit': 'rad/s'}
int Wheelspeedrpt7a::wheel_spd_front_right(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'name': 'wheel_spd_front_left', 'offset': 0.0,
// 'precision': 1.0, 'len': 16, 'is_signed_var': True, 'physical_range':
// '[-32768|32767]', 'bit': 7, 'type': 'int', 'order': 'motorola',
// 'physical_unit': 'rad/s'}
int Wheelspeedrpt7a::wheel_spd_front_left(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
