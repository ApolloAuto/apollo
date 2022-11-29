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

#include "modules/canbus_vehicle/lexus/protocol/headlight_aux_rpt_318.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Headlightauxrpt318::Headlightauxrpt318() {}
const int32_t Headlightauxrpt318::ID = 0x318;

void Headlightauxrpt318::Parse(const std::uint8_t* bytes, int32_t length,
                               Lexus* chassis) const {
  chassis->mutable_headlight_aux_rpt_318()->set_headlights_mode_is_valid(
      headlights_mode_is_valid(bytes, length));
  chassis->mutable_headlight_aux_rpt_318()->set_headlights_mode(
      headlights_mode(bytes, length));
  chassis->mutable_headlight_aux_rpt_318()->set_fog_lights_on_is_valid(
      fog_lights_on_is_valid(bytes, length));
  chassis->mutable_headlight_aux_rpt_318()->set_fog_lights_on(
      fog_lights_on(bytes, length));
  chassis->mutable_headlight_aux_rpt_318()->set_headlights_on_bright_is_valid(
      headlights_on_bright_is_valid(bytes, length));
  chassis->mutable_headlight_aux_rpt_318()->set_headlights_on_bright(
      headlights_on_bright(bytes, length));
  chassis->mutable_headlight_aux_rpt_318()->set_headlights_on_is_valid(
      headlights_on_is_valid(bytes, length));
  chassis->mutable_headlight_aux_rpt_318()->set_headlights_on(
      headlights_on(bytes, length));
}

// config detail: {'name': 'headlights_mode_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 19, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Headlightauxrpt318::headlights_mode_is_valid(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'headlights_mode', 'enum': {0:
// 'HEADLIGHTS_MODE_HEADLIGHTS_OFF', 1: 'HEADLIGHTS_MODE_PARKING_LIGHTS_ONLY',
// 2: 'HEADLIGHTS_MODE_HEADLIGHTS_ON_MANUAL_MODE', 3:
// 'HEADLIGHTS_MODE_HEADLIGHTS_ON_AUTO_MODE'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 15,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Headlight_aux_rpt_318::Headlights_modeType Headlightauxrpt318::headlights_mode(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Headlight_aux_rpt_318::Headlights_modeType ret =
      static_cast<Headlight_aux_rpt_318::Headlights_modeType>(x);
  return ret;
}

// config detail: {'name': 'fog_lights_on_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 18, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Headlightauxrpt318::fog_lights_on_is_valid(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'fog_lights_on', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Headlightauxrpt318::fog_lights_on(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'headlights_on_bright_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 17, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Headlightauxrpt318::headlights_on_bright_is_valid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'headlights_on_bright', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Headlightauxrpt318::headlights_on_bright(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'headlights_on_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 16, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Headlightauxrpt318::headlights_on_is_valid(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'headlights_on', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Headlightauxrpt318::headlights_on(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
