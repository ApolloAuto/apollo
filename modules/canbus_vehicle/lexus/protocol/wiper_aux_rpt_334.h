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

class Wiperauxrpt334 : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::Lexus> {
 public:
  static const int32_t ID;
  Wiperauxrpt334();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Lexus* chassis) const override;

 private:
  // config detail: {'name': 'SPRAY_EMPTY_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 13, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool spray_empty_is_valid(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'SPRAY_EMPTY', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 5,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool spray_empty(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SPRAY_NEAR_EMPTY_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 12, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool spray_near_empty_is_valid(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'name': 'SPRAY_NEAR_EMPTY', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 4, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool spray_near_empty(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'REAR_SPRAYING_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 11, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool rear_spraying_is_valid(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'REAR_SPRAYING', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 3,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool rear_spraying(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'REAR_WIPING_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 10, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool rear_wiping_is_valid(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'REAR_WIPING', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool rear_wiping(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'FRONT_SPRAYING_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 9, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool front_spraying_is_valid(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'name': 'FRONT_SPRAYING', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool front_spraying(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'FRONT_WIPING_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 8, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool front_wiping_is_valid(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'name': 'FRONT_WIPING', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool front_wiping(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
