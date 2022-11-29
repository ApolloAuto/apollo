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

#include "modules/canbus_vehicle/gem/protocol/steering_motor_rpt_2_74.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Steeringmotorrpt274::Steeringmotorrpt274() {}
const int32_t Steeringmotorrpt274::ID = 0x74;

void Steeringmotorrpt274::Parse(const std::uint8_t* bytes, int32_t length,
                                Gem* chassis) const {
  chassis->mutable_steering_motor_rpt_2_74()->set_encoder_temperature(
      encoder_temperature(bytes, length));
  chassis->mutable_steering_motor_rpt_2_74()->set_motor_temperature(
      motor_temperature(bytes, length));
  chassis->mutable_steering_motor_rpt_2_74()->set_angular_speed(
      angular_speed(bytes, length));
}

// config detail: {'name': 'encoder_temperature', 'offset': -40.0,
// 'precision': 1.0, 'len': 16, 'is_signed_var': True, 'physical_range':
// '[-32808|32727]', 'bit': 7, 'type': 'int', 'order': 'motorola',
// 'physical_unit': 'deg C'}
int Steeringmotorrpt274::encoder_temperature(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x + -40;
  return ret;
}

// config detail: {'name': 'motor_temperature', 'offset': -40.0,
// 'precision': 1.0, 'len': 16, 'is_signed_var': True, 'physical_range':
// '[-32808|32727]', 'bit': 23, 'type': 'int', 'order': 'motorola',
// 'physical_unit': 'deg C'}
int Steeringmotorrpt274::motor_temperature(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x + -40;
  return ret;
}

// config detail: {'name': 'angular_speed', 'offset': 0.0, 'precision': 0.001,
// 'len': 32, 'is_signed_var': True, 'physical_range':
// '[-2147483.648|2147483.647]', 'bit': 39, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'rev/s'}
double Steeringmotorrpt274::angular_speed(const std::uint8_t* bytes,
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
