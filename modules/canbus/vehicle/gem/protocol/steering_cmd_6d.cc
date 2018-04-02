/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/gem/protocol/steering_cmd_6d.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Steeringcmd6d::Steeringcmd6d() {}
const int32_t Steeringcmd6d::ID = 0x6D;

void Steeringcmd6d::Parse(const std::uint8_t* bytes, int32_t length,
                          ChassisDetail* chassis) const {
  chassis->mutable_gem()->mutable_steering_cmd_6d()->set_position_value(
      position_value(bytes, length));
  chassis->mutable_gem()->mutable_steering_cmd_6d()->set_speed_limit(
      speed_limit(bytes, length));
}

// config detail: {'name': 'position_value', 'offset': 0.0, 'precision': 0.001,
// 'len': 32, 'is_signed_var': True, 'physical_range':
// '[-2147483.648|2147483.647]', 'bit': 7, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'radians'}
double Steeringcmd6d::position_value(const std::uint8_t* bytes,
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

  x <<= 0;
  x >>= 0;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'speed_limit', 'offset': 0.0, 'precision': 0.001,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit': 39,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'rad/s'}
double Steeringcmd6d::speed_limit(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
