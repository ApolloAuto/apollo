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

#include "modules/canbus_vehicle/lexus/protocol/shift_aux_rpt_328.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Shiftauxrpt328::Shiftauxrpt328() {}
const int32_t Shiftauxrpt328::ID = 0x328;

void Shiftauxrpt328::Parse(const std::uint8_t* bytes, int32_t length,
                           Lexus* chassis) const {
  chassis->mutable_shift_aux_rpt_328()->set_speed_interlock_active_is_valid(
      speed_interlock_active_is_valid(bytes, length));
  chassis->mutable_shift_aux_rpt_328()->set_speed_interlock_active(
      speed_interlock_active(bytes, length));
  chassis->mutable_shift_aux_rpt_328()->set_brake_interlock_active_is_valid(
      brake_interlock_active_is_valid(bytes, length));
  chassis->mutable_shift_aux_rpt_328()->set_brake_interlock_active(
      brake_interlock_active(bytes, length));
  chassis->mutable_shift_aux_rpt_328()->set_stay_in_neutral_mode_is_valid(
      stay_in_neutral_mode_is_valid(bytes, length));
  chassis->mutable_shift_aux_rpt_328()->set_stay_in_neutral_mode(
      stay_in_neutral_mode(bytes, length));
  chassis->mutable_shift_aux_rpt_328()->set_between_gears_is_valid(
      between_gears_is_valid(bytes, length));
  chassis->mutable_shift_aux_rpt_328()->set_between_gears(
      between_gears(bytes, length));
}

// config detail: {'name': 'speed_interlock_active_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 11, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Shiftauxrpt328::speed_interlock_active_is_valid(const std::uint8_t* bytes,
                                                     int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'speed_interlock_active', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 3, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Shiftauxrpt328::speed_interlock_active(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'brake_interlock_active_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 10, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Shiftauxrpt328::brake_interlock_active_is_valid(const std::uint8_t* bytes,
                                                     int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'brake_interlock_active', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 2, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Shiftauxrpt328::brake_interlock_active(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'stay_in_neutral_mode_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 9, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Shiftauxrpt328::stay_in_neutral_mode_is_valid(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'stay_in_neutral_mode', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Shiftauxrpt328::stay_in_neutral_mode(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'between_gears_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 8, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Shiftauxrpt328::between_gears_is_valid(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'between_gears', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Shiftauxrpt328::between_gears(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
