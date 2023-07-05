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

#include "modules/canbus_vehicle/lexus/proto/lexus.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace lexus {

class Interiorlightsrpt416 : public ::apollo::drivers::canbus::ProtocolData<
                                 ::apollo::canbus::Lexus> {
 public:
  static const int32_t ID;
  Interiorlightsrpt416();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Lexus* chassis) const override;

 private:
  // config detail: {'name': 'DIM_LEVEL_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 19, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool dim_level_is_valid(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'name': 'MOOD_LIGHTS_ON_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 18, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool mood_lights_on_is_valid(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'name': 'REAR_DOME_LIGHTS_ON_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 17, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool rear_dome_lights_on_is_valid(const std::uint8_t* bytes,
                                    const int32_t length) const;

  // config detail: {'name': 'FRONT_DOME_LIGHTS_ON_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 16, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool front_dome_lights_on_is_valid(const std::uint8_t* bytes,
                                     const int32_t length) const;

  // config detail: {'name': 'DIM_LEVEL', 'enum': {0: 'DIM_LEVEL_DIM_LEVEL_MIN',
  // 1: 'DIM_LEVEL_DIM_LEVEL_1', 2: 'DIM_LEVEL_DIM_LEVEL_2', 3:
  // 'DIM_LEVEL_DIM_LEVEL_3', 4: 'DIM_LEVEL_DIM_LEVEL_4', 5:
  // 'DIM_LEVEL_DIM_LEVEL_5', 6: 'DIM_LEVEL_DIM_LEVEL_6', 7:
  // 'DIM_LEVEL_DIM_LEVEL_7', 8: 'DIM_LEVEL_DIM_LEVEL_8', 9:
  // 'DIM_LEVEL_DIM_LEVEL_9', 10: 'DIM_LEVEL_DIM_LEVEL_10', 11:
  // 'DIM_LEVEL_DIM_LEVEL_11', 12: 'DIM_LEVEL_DIM_LEVEL_MAX'}, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range':
  // '[0|12]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit':
  // ''}
  Interior_lights_rpt_416::Dim_levelType dim_level(const std::uint8_t* bytes,
                                                   const int32_t length) const;

  // config detail: {'name': 'MOOD_LIGHTS_ON', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool mood_lights_on(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'REAR_DOME_LIGHTS_ON', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool rear_dome_lights_on(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'name': 'FRONT_DOME_LIGHTS_ON', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 0, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool front_dome_lights_on(const std::uint8_t* bytes,
                            const int32_t length) const;
};

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
