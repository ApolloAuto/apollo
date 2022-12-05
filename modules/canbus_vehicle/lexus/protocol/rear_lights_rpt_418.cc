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

#include "modules/canbus_vehicle/lexus/protocol/rear_lights_rpt_418.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Rearlightsrpt418::Rearlightsrpt418() {}
const int32_t Rearlightsrpt418::ID = 0x418;

void Rearlightsrpt418::Parse(const std::uint8_t* bytes, int32_t length,
                             Lexus* chassis) const {
  chassis->mutable_rear_lights_rpt_418()->set_reverse_lights_on_is_valid(
      reverse_lights_on_is_valid(bytes, length));
  chassis->mutable_rear_lights_rpt_418()->set_brake_lights_on_is_valid(
      brake_lights_on_is_valid(bytes, length));
  chassis->mutable_rear_lights_rpt_418()->set_reverse_lights_on(
      reverse_lights_on(bytes, length));
  chassis->mutable_rear_lights_rpt_418()->set_brake_lights_on(
      brake_lights_on(bytes, length));
}

// config detail: {'name': 'reverse_lights_on_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 9, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Rearlightsrpt418::reverse_lights_on_is_valid(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'brake_lights_on_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 8, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Rearlightsrpt418::brake_lights_on_is_valid(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'reverse_lights_on', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Rearlightsrpt418::reverse_lights_on(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'brake_lights_on', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Rearlightsrpt418::brake_lights_on(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
