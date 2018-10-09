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

#pragma once

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace lexus {

class Headlightauxrpt318 : public ::apollo::drivers::canbus::ProtocolData<
                               ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Headlightauxrpt318();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'HEADLIGHTS_MODE_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 19, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool headlights_mode_is_valid(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'name': 'HEADLIGHTS_MODE', 'enum': {0:
  // 'HEADLIGHTS_MODE_HEADLIGHTS_OFF', 1: 'HEADLIGHTS_MODE_PARKING_LIGHTS_ONLY',
  // 2: 'HEADLIGHTS_MODE_HEADLIGHTS_ON_MANUAL_MODE', 3:
  // 'HEADLIGHTS_MODE_HEADLIGHTS_ON_AUTO_MODE'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit':
  // 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Headlight_aux_rpt_318::Headlights_modeType headlights_mode(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'FOG_LIGHTS_ON_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 18, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool fog_lights_on_is_valid(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'FOG_LIGHTS_ON', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool fog_lights_on(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'HEADLIGHTS_ON_BRIGHT_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 17, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool headlights_on_bright_is_valid(const std::uint8_t* bytes,
                                     const int32_t length) const;

  // config detail: {'name': 'HEADLIGHTS_ON_BRIGHT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool headlights_on_bright(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'HEADLIGHTS_ON_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 16, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool headlights_on_is_valid(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'HEADLIGHTS_ON', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool headlights_on(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
