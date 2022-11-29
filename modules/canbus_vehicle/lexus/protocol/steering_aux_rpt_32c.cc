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

#include "modules/canbus_vehicle/lexus/protocol/steering_aux_rpt_32c.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Steeringauxrpt32c::Steeringauxrpt32c() {}
const int32_t Steeringauxrpt32c::ID = 0x32C;

void Steeringauxrpt32c::Parse(const std::uint8_t* bytes, int32_t length,
                              Lexus* chassis) const {
  chassis->mutable_steering_aux_rpt_32c()->set_user_interaction_is_valid(
      user_interaction_is_valid(bytes, length));
  chassis->mutable_steering_aux_rpt_32c()->set_user_interaction(
      user_interaction(bytes, length));
  chassis->mutable_steering_aux_rpt_32c()->set_rotation_rate_is_valid(
      rotation_rate_is_valid(bytes, length));
  chassis->mutable_steering_aux_rpt_32c()->set_rotation_rate(
      rotation_rate(bytes, length));
  chassis->mutable_steering_aux_rpt_32c()->set_raw_torque_is_valid(
      raw_torque_is_valid(bytes, length));
  chassis->mutable_steering_aux_rpt_32c()->set_raw_torque(
      raw_torque(bytes, length));
  chassis->mutable_steering_aux_rpt_32c()->set_raw_position_is_valid(
      raw_position_is_valid(bytes, length));
  chassis->mutable_steering_aux_rpt_32c()->set_raw_position(
      raw_position(bytes, length));
}

// config detail: {'name': 'user_interaction_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 59, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Steeringauxrpt32c::user_interaction_is_valid(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'user_interaction', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 48,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Steeringauxrpt32c::user_interaction(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rotation_rate_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 58, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Steeringauxrpt32c::rotation_rate_is_valid(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rotation_rate', 'offset': 0.0, 'precision': 0.001,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit': 39,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'rad/s'}
double Steeringauxrpt32c::rotation_rate(const std::uint8_t* bytes,
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

// config detail: {'name': 'raw_torque_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 57, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Steeringauxrpt32c::raw_torque_is_valid(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'raw_torque', 'offset': 0.0, 'precision': 0.001,
// 'len': 16, 'is_signed_var': True, 'physical_range': '[-32.768|32.767]',
// 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Steeringauxrpt32c::raw_torque(const std::uint8_t* bytes,
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

// config detail: {'name': 'raw_position_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 56, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Steeringauxrpt32c::raw_position_is_valid(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'raw_position', 'offset': 0.0, 'precision': 0.001,
// 'len': 16, 'is_signed_var': True, 'physical_range': '[-32.768|32.767]',
// 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Steeringauxrpt32c::raw_position(const std::uint8_t* bytes,
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
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
