/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_VEHICLE1_4F0_H_
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_VEHICLE1_4F0_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::DelphiESR;

class Vehicle14f0 : public apollo::drivers::canbus::ProtocolData<DelphiESR> {
 public:
  static const int32_t ID;
  Vehicle14f0();
  void Parse(const std::uint8_t* bytes, int32_t length,
             DelphiESR* delphi_esr) const override;

 private:
  // config detail: {'name': 'CAN_RX_STEERING_ANGLE_VALIDITY', 'enum': {0:
  // 'CAN_RX_STEERING_ANGLE_VALIDITY_INVALID', 1:
  // 'CAN_RX_STEERING_ANGLE_VALIDITY_VALID'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle1_4f0::Can_rx_steering_angle_validityType
  can_rx_steering_angle_validity(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'name': 'CAN_RX_STEERING_ANGLE_RATE', 'offset': 0.0,
  // 'precision': 1.0, 'len': 11, 'is_signed_var': False, 'physical_range':
  // '[0|2047]', 'bit': 50, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // 'deg/s'}
  int can_rx_steering_angle_rate(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'name': 'CAN_RX_STEERING_ANGLE_SIGN', 'enum': {0:
  // 'CAN_RX_STEERING_ANGLE_SIGN_COUNTERCLOCKWISE', 1:
  // 'CAN_RX_STEERING_ANGLE_SIGN_CLOCKWISE'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 46, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle1_4f0::Can_rx_steering_angle_signType can_rx_steering_angle_sign(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_STEERING_ANGLE_RATE_SIGN', 'enum': {0:
  // 'CAN_RX_STEERING_ANGLE_RATE_SIGN_COUNTERCLOCKWISE', 1:
  // 'CAN_RX_STEERING_ANGLE_RATE_SIGN_CLOCKWISE'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 30, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle1_4f0::Can_rx_steering_angle_rate_signType
  can_rx_steering_angle_rate_sign(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'name': 'CAN_RX_STEERING_ANGLE', 'offset': 0.0,
  // 'precision': 1.0, 'len': 11, 'is_signed_var': False, 'physical_range':
  // '[0|2047]', 'bit': 45, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // 'deg'}
  int can_rx_steering_angle(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'CAN_RX_RADIUS_CURVATURE', 'offset': 0.0,
  // 'precision': 1.0, 'len': 14, 'is_signed_var': True, 'physical_range':
  // '[-8192|8191]', 'bit': 29, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'm'}
  int can_rx_radius_curvature(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'CAN_RX_YAW_RATE_VALIDITY', 'enum': {0:
  // 'CAN_RX_YAW_RATE_VALIDITY_INVALID', 1: 'CAN_RX_YAW_RATE_VALIDITY_VALID'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 31, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Vehicle1_4f0::Can_rx_yaw_rate_validityType can_rx_yaw_rate_validity(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_YAW_RATE', 'offset': 0.0, 'precision':
  // 0.0625, 'len': 12, 'is_signed_var': True, 'physical_range':
  // '[-128|127.9375]', 'bit': 11, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'deg/s'}
  double can_rx_yaw_rate(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_VEHICLE_SPEED_DIRECTION', 'enum': {0:
  // 'CAN_RX_VEHICLE_SPEED_DIRECTION_FORWARD', 1:
  // 'CAN_RX_VEHICLE_SPEED_DIRECTION_REVERSE'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle1_4f0::Can_rx_vehicle_speed_directionType
  can_rx_vehicle_speed_direction(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'name': 'CAN_RX_VEHICLE_SPEED', 'offset': 0.0, 'precision':
  // 0.0625, 'len': 11, 'is_signed_var': False, 'physical_range':
  // '[0|127.9375]', 'bit': 7, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm/s'}
  double can_rx_vehicle_speed(const std::uint8_t* bytes,
                              const int32_t length) const;
};

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_VEHICLE1_4F0_H_
