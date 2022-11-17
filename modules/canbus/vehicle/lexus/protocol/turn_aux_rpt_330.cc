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

#include "modules/canbus/vehicle/lexus/protocol/turn_aux_rpt_330.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Turnauxrpt330::Turnauxrpt330() {}
const int32_t Turnauxrpt330::ID = 0x330;

void Turnauxrpt330::Parse(const std::uint8_t* bytes, int32_t length,
                          ChassisDetail* chassis) const {
  chassis->mutable_lexus()
      ->mutable_turn_aux_rpt_330()
      ->set_pass_blinker_bulb_on_is_valid(
          pass_blinker_bulb_on_is_valid(bytes, length));
  chassis->mutable_lexus()
      ->mutable_turn_aux_rpt_330()
      ->set_pass_blinker_bulb_on(pass_blinker_bulb_on(bytes, length));
  chassis->mutable_lexus()
      ->mutable_turn_aux_rpt_330()
      ->set_driver_blinker_bulb_on_is_valid(
          driver_blinker_bulb_on_is_valid(bytes, length));
  chassis->mutable_lexus()
      ->mutable_turn_aux_rpt_330()
      ->set_driver_blinker_bulb_on(driver_blinker_bulb_on(bytes, length));
}

// config detail: {'name': 'pass_blinker_bulb_on_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 9, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Turnauxrpt330::pass_blinker_bulb_on_is_valid(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'pass_blinker_bulb_on', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Turnauxrpt330::pass_blinker_bulb_on(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'driver_blinker_bulb_on_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 8, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Turnauxrpt330::driver_blinker_bulb_on_is_valid(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'driver_blinker_bulb_on', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 0, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Turnauxrpt330::driver_blinker_bulb_on(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
