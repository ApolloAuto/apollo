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

#include "modules/canbus/vehicle/gem/protocol/brake_rpt_6c.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Brakerpt6c::Brakerpt6c() {}
const int32_t Brakerpt6c::ID = 0x6C;

void Brakerpt6c::Parse(const std::uint8_t* bytes, int32_t length,
                       ChassisDetail* chassis) const {
  chassis->mutable_gem()->mutable_brake_rpt_6c()->set_manual_input(
      manual_input(bytes, length));
  chassis->mutable_gem()->mutable_brake_rpt_6c()->set_commanded_value(
      commanded_value(bytes, length));
  chassis->mutable_gem()->mutable_brake_rpt_6c()->set_output_value(
      output_value(bytes, length));
  chassis->mutable_gem()->mutable_brake_rpt_6c()->set_brake_on_off(
      brake_on_off(bytes, length));
}

// config detail: {'name': 'manual_input', 'offset': 0.0, 'precision': 0.001,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 7,
// 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
double Brakerpt6c::manual_input(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'commanded_value', 'offset': 0.0, 'precision': 0.001,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 23,
// 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
double Brakerpt6c::commanded_value(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'output_value', 'offset': 0.0, 'precision': 0.001,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 39,
// 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
double Brakerpt6c::output_value(const std::uint8_t* bytes,
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

// config detail: {'name': 'brake_on_off', 'enum': {0: 'BRAKE_ON_OFF_OFF', 1:
// 'BRAKE_ON_OFF_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 48, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Brake_rpt_6c::Brake_on_offType Brakerpt6c::brake_on_off(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 1);

  Brake_rpt_6c::Brake_on_offType ret =
      static_cast<Brake_rpt_6c::Brake_on_offType>(x);
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
