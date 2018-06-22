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

#ifndef MODULES_CANBUS_VEHICLE_GEM_PROTOCOL_WHEEL_SPEED_RPT_7A_H_
#define MODULES_CANBUS_VEHICLE_GEM_PROTOCOL_WHEEL_SPEED_RPT_7A_H_

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace gem {

class Wheelspeedrpt7a : public ::apollo::drivers::canbus::ProtocolData<
                            ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Wheelspeedrpt7a();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'WHEEL_SPD_REAR_RIGHT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 16, 'is_signed_var': True, 'physical_range':
  // '[-32768|32767]', 'bit': 55, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'rad/s'}
  int wheel_spd_rear_right(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'name': 'WHEEL_SPD_REAR_LEFT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 16, 'is_signed_var': True, 'physical_range':
  // '[-32768|32767]', 'bit': 39, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'rad/s'}
  int wheel_spd_rear_left(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'name': 'WHEEL_SPD_FRONT_RIGHT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 16, 'is_signed_var': True, 'physical_range':
  // '[-32768|32767]', 'bit': 23, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'rad/s'}
  int wheel_spd_front_right(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'WHEEL_SPD_FRONT_LEFT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 16, 'is_signed_var': True, 'physical_range':
  // '[-32768|32767]', 'bit': 7, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'rad/s'}
  int wheel_spd_front_left(const std::uint8_t* bytes,
                           const int32_t length) const;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_GEM_PROTOCOL_WHEEL_SPEED_RPT_7A_H_
