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

#include "modules/canbus_vehicle/gem/protocol/steering_rpt_1_6e.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Steeringrpt16e::Steeringrpt16e() {}
const int32_t Steeringrpt16e::ID = 0x6E;

void Steeringrpt16e::Parse(const std::uint8_t* bytes, int32_t length,
                           Gem* chassis) const {
  chassis->mutable_steering_rpt_1_6e()->set_manual_input(
      manual_input(bytes, length));
  chassis->mutable_steering_rpt_1_6e()->set_commanded_value(
      commanded_value(bytes, length));
  chassis->mutable_steering_rpt_1_6e()->set_output_value(
      output_value(bytes, length));
}

// config detail: {'name': 'manual_input', 'offset': 0.0, 'precision': 0.001,
// 'len': 16, 'is_signed_var': True, 'physical_range': '[-32.768|32.767]',
// 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rad/s'}
double Steeringrpt16e::manual_input(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'commanded_value', 'offset': 0.0, 'precision': 0.001,
// 'len': 16, 'is_signed_var': True, 'physical_range': '[-32.768|32.767]',
// 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rad/s'}
double Steeringrpt16e::commanded_value(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'output_value', 'offset': 0.0, 'precision': 0.001,
// 'len': 16, 'is_signed_var': True, 'physical_range': '[-32.768|32.767]',
// 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rad/s'}
double Steeringrpt16e::output_value(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
