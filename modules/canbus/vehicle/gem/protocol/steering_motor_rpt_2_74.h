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

#ifndef MODULES_CANBUS_VEHICLE_GEM_PROTOCOL_STEERING_MOTOR_RPT_2_74_H_
#define MODULES_CANBUS_VEHICLE_GEM_PROTOCOL_STEERING_MOTOR_RPT_2_74_H_

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace gem {

class Steeringmotorrpt274 : public ::apollo::drivers::canbus::ProtocolData<
                                ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Steeringmotorrpt274();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'ENCODER_TEMPERATURE', 'offset': -40.0,
  // 'precision': 1.0, 'len': 16, 'is_signed_var': True, 'physical_range':
  // '[-32808|32727]', 'bit': 7, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'deg C'}
  int encoder_temperature(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'name': 'MOTOR_TEMPERATURE', 'offset': -40.0,
  // 'precision': 1.0, 'len': 16, 'is_signed_var': True, 'physical_range':
  // '[-32808|32727]', 'bit': 23, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'deg C'}
  int motor_temperature(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ANGULAR_SPEED', 'offset': 0.0, 'precision': 0.001,
  // 'len': 32, 'is_signed_var': True, 'physical_range':
  // '[-2147483.648|2147483.647]', 'bit': 39, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'rev/s'}
  double angular_speed(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_GEM_PROTOCOL_STEERING_MOTOR_RPT_2_74_H_
