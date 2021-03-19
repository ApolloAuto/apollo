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

#include "modules/canbus/vehicle/lexus/protocol/brake_aux_rpt_304.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Brakeauxrpt304::Brakeauxrpt304() {}
const int32_t Brakeauxrpt304::ID = 0x304;

void Brakeauxrpt304::Parse(const std::uint8_t* bytes, int32_t length,
                           ChassisDetail* chassis) const {
  chassis->mutable_lexus()
      ->mutable_brake_aux_rpt_304()
      ->set_brake_on_off_is_valid(brake_on_off_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_brake_aux_rpt_304()->set_brake_on_off(
      brake_on_off(bytes, length));
  chassis->mutable_lexus()
      ->mutable_brake_aux_rpt_304()
      ->set_user_interaction_is_valid(user_interaction_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_brake_aux_rpt_304()->set_user_interaction(
      user_interaction(bytes, length));
  chassis->mutable_lexus()
      ->mutable_brake_aux_rpt_304()
      ->set_raw_brake_pressure_is_valid(
          raw_brake_pressure_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_brake_aux_rpt_304()->set_raw_brake_pressure(
      raw_brake_pressure(bytes, length));
  chassis->mutable_lexus()
      ->mutable_brake_aux_rpt_304()
      ->set_raw_pedal_force_is_valid(raw_pedal_force_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_brake_aux_rpt_304()->set_raw_pedal_force(
      raw_pedal_force(bytes, length));
  chassis->mutable_lexus()
      ->mutable_brake_aux_rpt_304()
      ->set_raw_pedal_pos_is_valid(raw_pedal_pos_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_brake_aux_rpt_304()->set_raw_pedal_pos(
      raw_pedal_pos(bytes, length));
}

// config detail: {'name': 'brake_on_off_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 60, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Brakeauxrpt304::brake_on_off_is_valid(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'brake_on_off', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 49,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Brakeauxrpt304::brake_on_off(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'user_interaction_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 59, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Brakeauxrpt304::user_interaction_is_valid(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'user_interaction', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 48,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Brakeauxrpt304::user_interaction(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'raw_brake_pressure_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 58, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Brakeauxrpt304::raw_brake_pressure_is_valid(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'raw_brake_pressure', 'offset': 0.0,
// 'precision': 1.0, 'len': 16, 'is_signed_var': True, 'physical_range':
// '[-32.768|32.767]', 'bit': 39, 'type': 'double', 'order': 'motorola',
// 'physical_unit': ''}
double Brakeauxrpt304::raw_brake_pressure(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x;
  return ret;
}

// config detail: {'name': 'raw_pedal_force_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 57, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Brakeauxrpt304::raw_pedal_force_is_valid(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'raw_pedal_force', 'offset': 0.0, 'precision': 1.0,
// 'len': 16, 'is_signed_var': True, 'physical_range': '[-32.768|32.767]',
// 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Brakeauxrpt304::raw_pedal_force(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x;
  return ret;
}

// config detail: {'name': 'raw_pedal_pos_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 56, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Brakeauxrpt304::raw_pedal_pos_is_valid(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'raw_pedal_pos', 'offset': 0.0, 'precision': 1.0,
// 'len': 16, 'is_signed_var': True, 'physical_range': '[-32.768|32.767]',
// 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Brakeauxrpt304::raw_pedal_pos(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
