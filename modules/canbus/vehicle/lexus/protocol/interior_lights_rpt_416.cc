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

#include "modules/canbus/vehicle/lexus/protocol/interior_lights_rpt_416.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Interiorlightsrpt416::Interiorlightsrpt416() {}
const int32_t Interiorlightsrpt416::ID = 0x416;

void Interiorlightsrpt416::Parse(const std::uint8_t* bytes, int32_t length,
                                 ChassisDetail* chassis) const {
  chassis->mutable_lexus()
      ->mutable_interior_lights_rpt_416()
      ->set_dim_level_is_valid(dim_level_is_valid(bytes, length));
  chassis->mutable_lexus()
      ->mutable_interior_lights_rpt_416()
      ->set_mood_lights_on_is_valid(mood_lights_on_is_valid(bytes, length));
  chassis->mutable_lexus()
      ->mutable_interior_lights_rpt_416()
      ->set_rear_dome_lights_on_is_valid(
          rear_dome_lights_on_is_valid(bytes, length));
  chassis->mutable_lexus()
      ->mutable_interior_lights_rpt_416()
      ->set_front_dome_lights_on_is_valid(
          front_dome_lights_on_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_interior_lights_rpt_416()->set_dim_level(
      dim_level(bytes, length));
  chassis->mutable_lexus()
      ->mutable_interior_lights_rpt_416()
      ->set_mood_lights_on(mood_lights_on(bytes, length));
  chassis->mutable_lexus()
      ->mutable_interior_lights_rpt_416()
      ->set_rear_dome_lights_on(rear_dome_lights_on(bytes, length));
  chassis->mutable_lexus()
      ->mutable_interior_lights_rpt_416()
      ->set_front_dome_lights_on(front_dome_lights_on(bytes, length));
}

// config detail: {'name': 'dim_level_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 19, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Interiorlightsrpt416::dim_level_is_valid(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'mood_lights_on_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 18, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Interiorlightsrpt416::mood_lights_on_is_valid(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rear_dome_lights_on_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 17, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Interiorlightsrpt416::rear_dome_lights_on_is_valid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'front_dome_lights_on_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 16, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Interiorlightsrpt416::front_dome_lights_on_is_valid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'dim_level', 'enum': {0: 'DIM_LEVEL_DIM_LEVEL_MIN',
// 1: 'DIM_LEVEL_DIM_LEVEL_1', 2: 'DIM_LEVEL_DIM_LEVEL_2', 3:
// 'DIM_LEVEL_DIM_LEVEL_3', 4: 'DIM_LEVEL_DIM_LEVEL_4', 5:
// 'DIM_LEVEL_DIM_LEVEL_5', 6: 'DIM_LEVEL_DIM_LEVEL_6', 7:
// 'DIM_LEVEL_DIM_LEVEL_7', 8: 'DIM_LEVEL_DIM_LEVEL_8', 9:
// 'DIM_LEVEL_DIM_LEVEL_9', 10: 'DIM_LEVEL_DIM_LEVEL_10', 11:
// 'DIM_LEVEL_DIM_LEVEL_11', 12: 'DIM_LEVEL_DIM_LEVEL_MAX'}, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|12]',
// 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Interior_lights_rpt_416::Dim_levelType Interiorlightsrpt416::dim_level(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Interior_lights_rpt_416::Dim_levelType ret =
      static_cast<Interior_lights_rpt_416::Dim_levelType>(x);
  return ret;
}

// config detail: {'name': 'mood_lights_on', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Interiorlightsrpt416::mood_lights_on(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rear_dome_lights_on', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Interiorlightsrpt416::rear_dome_lights_on(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'front_dome_lights_on', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 0, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Interiorlightsrpt416::front_dome_lights_on(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
