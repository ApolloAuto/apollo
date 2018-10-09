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

class Steeringauxrpt32c : public ::apollo::drivers::canbus::ProtocolData<
                              ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Steeringauxrpt32c();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'USER_INTERACTION_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 59, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool user_interaction_is_valid(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'name': 'USER_INTERACTION', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 48, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool user_interaction(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ROTATION_RATE_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 58, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool rotation_rate_is_valid(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'ROTATION_RATE', 'offset': 0.0, 'precision': 0.001,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit':
  // 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rad/s'}
  double rotation_rate(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'RAW_TORQUE_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 57, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool raw_torque_is_valid(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'name': 'RAW_TORQUE', 'offset': 0.0, 'precision': 0.001,
  // 'len': 16, 'is_signed_var': True, 'physical_range': '[-32.768|32.767]',
  // 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  double raw_torque(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'RAW_POSITION_IS_VALID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 56, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool raw_position_is_valid(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'name': 'RAW_POSITION', 'offset': 0.0, 'precision': 0.001,
  // 'len': 16, 'is_signed_var': True, 'physical_range': '[-32.768|32.767]',
  // 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  double raw_position(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
