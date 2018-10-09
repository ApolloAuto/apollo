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

class Wheelspeedrpt407 : public ::apollo::drivers::canbus::ProtocolData<
                             ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Wheelspeedrpt407();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'WHEEL_SPD_REAR_RIGHT', 'offset': 0.0, 'precision':
  // 0.01, 'len': 16, 'is_signed_var': True, 'physical_range':
  // '[-327.68|327.67]', 'bit': 55, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'rad/s'}
  double wheel_spd_rear_right(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'WHEEL_SPD_REAR_LEFT', 'offset': 0.0, 'precision':
  // 0.01, 'len': 16, 'is_signed_var': True, 'physical_range':
  // '[-327.68|327.67]', 'bit': 39, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'rad/s'}
  double wheel_spd_rear_left(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'name': 'WHEEL_SPD_FRONT_RIGHT', 'offset': 0.0,
  // 'precision': 0.01, 'len': 16, 'is_signed_var': True, 'physical_range':
  // '[-327.68|327.67]', 'bit': 23, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'rad/s'}
  double wheel_spd_front_right(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'name': 'WHEEL_SPD_FRONT_LEFT', 'offset': 0.0, 'precision':
  // 0.01, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[-327.68|327.67]', 'bit': 7, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'rad/s'}
  double wheel_spd_front_left(const std::uint8_t* bytes,
                              const int32_t length) const;
};

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
