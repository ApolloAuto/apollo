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

class Shiftauxrpt328 : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Shiftauxrpt328();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'SPEED_INTERLOCK_ACTIVE_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 11, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool speed_interlock_active_is_valid(const std::uint8_t* bytes,
                                       const int32_t length) const;

  // config detail: {'name': 'SPEED_INTERLOCK_ACTIVE', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 3, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool speed_interlock_active(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'BRAKE_INTERLOCK_ACTIVE_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 10, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool brake_interlock_active_is_valid(const std::uint8_t* bytes,
                                       const int32_t length) const;

  // config detail: {'name': 'BRAKE_INTERLOCK_ACTIVE', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 2, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool brake_interlock_active(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'STAY_IN_NEUTRAL_MODE_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 9, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool stay_in_neutral_mode_is_valid(const std::uint8_t* bytes,
                                     const int32_t length) const;

  // config detail: {'name': 'STAY_IN_NEUTRAL_MODE', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool stay_in_neutral_mode(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'BETWEEN_GEARS_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 8, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool between_gears_is_valid(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'BETWEEN_GEARS', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool between_gears(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
