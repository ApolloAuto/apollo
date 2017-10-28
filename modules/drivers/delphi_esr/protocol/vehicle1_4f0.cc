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

#include "modules/drivers/delphi_esr/protocol/vehicle1_4f0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Vehicle14f0::Vehicle14f0() {}
const int32_t Vehicle14f0::ID = 0x4F0;

void Vehicle14f0::Parse(const std::uint8_t* bytes, int32_t length,
                        DelphiESR* delphi_esr) const {
  delphi_esr->mutable_vehicle1_4f0()->set_can_rx_steering_angle_validity(
      can_rx_steering_angle_validity(bytes, length));
  delphi_esr->mutable_vehicle1_4f0()->set_can_rx_steering_angle_rate(
      can_rx_steering_angle_rate(bytes, length));
  delphi_esr->mutable_vehicle1_4f0()->set_can_rx_steering_angle_sign(
      can_rx_steering_angle_sign(bytes, length));
  delphi_esr->mutable_vehicle1_4f0()->set_can_rx_steering_angle_rate_sign(
      can_rx_steering_angle_rate_sign(bytes, length));
  delphi_esr->mutable_vehicle1_4f0()->set_can_rx_steering_angle(
      can_rx_steering_angle(bytes, length));
  delphi_esr->mutable_vehicle1_4f0()->set_can_rx_radius_curvature(
      can_rx_radius_curvature(bytes, length));
  delphi_esr->mutable_vehicle1_4f0()->set_can_rx_yaw_rate_validity(
      can_rx_yaw_rate_validity(bytes, length));
  delphi_esr->mutable_vehicle1_4f0()->set_can_rx_yaw_rate(
      can_rx_yaw_rate(bytes, length));
  delphi_esr->mutable_vehicle1_4f0()->set_can_rx_vehicle_speed_direction(
      can_rx_vehicle_speed_direction(bytes, length));
  delphi_esr->mutable_vehicle1_4f0()->set_can_rx_vehicle_speed(
      can_rx_vehicle_speed(bytes, length));
}

// config detail: {'name': 'can_rx_steering_angle_validity', 'enum': {0:
// 'CAN_RX_STEERING_ANGLE_VALIDITY_INVALID', 1:
// 'CAN_RX_STEERING_ANGLE_VALIDITY_VALID'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 47,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Vehicle1_4f0::Can_rx_steering_angle_validityType
Vehicle14f0::can_rx_steering_angle_validity(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(7, 1);

  Vehicle1_4f0::Can_rx_steering_angle_validityType ret =
      static_cast<Vehicle1_4f0::Can_rx_steering_angle_validityType>(x);
  return ret;
}

// config detail: {'name': 'can_rx_steering_angle_rate', 'offset': 0.0,
// 'precision': 1.0, 'len': 11, 'is_signed_var': False, 'physical_range':
// '[0|2047]', 'bit': 50, 'type': 'int', 'order': 'motorola', 'physical_unit':
// 'deg/s'}
int Vehicle14f0::can_rx_steering_angle_rate(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_rx_steering_angle_sign', 'enum': {0:
// 'CAN_RX_STEERING_ANGLE_SIGN_COUNTERCLOCKWISE', 1:
// 'CAN_RX_STEERING_ANGLE_SIGN_CLOCKWISE'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 46,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Vehicle1_4f0::Can_rx_steering_angle_signType
Vehicle14f0::can_rx_steering_angle_sign(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(6, 1);

  Vehicle1_4f0::Can_rx_steering_angle_signType ret =
      static_cast<Vehicle1_4f0::Can_rx_steering_angle_signType>(x);
  return ret;
}

// config detail: {'name': 'can_rx_steering_angle_rate_sign', 'enum': {0:
// 'CAN_RX_STEERING_ANGLE_RATE_SIGN_COUNTERCLOCKWISE', 1:
// 'CAN_RX_STEERING_ANGLE_RATE_SIGN_CLOCKWISE'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 30,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Vehicle1_4f0::Can_rx_steering_angle_rate_signType
Vehicle14f0::can_rx_steering_angle_rate_sign(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(6, 1);

  Vehicle1_4f0::Can_rx_steering_angle_rate_signType ret =
      static_cast<Vehicle1_4f0::Can_rx_steering_angle_rate_signType>(x);
  return ret;
}

// config detail: {'name': 'can_rx_steering_angle', 'offset': 0.0, 'precision':
// 1.0, 'len': 11, 'is_signed_var': False, 'physical_range': '[0|2047]', 'bit':
// 45, 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg'}
int Vehicle14f0::can_rx_steering_angle(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_rx_radius_curvature', 'offset': 0.0,
// 'precision': 1.0, 'len': 14, 'is_signed_var': True, 'physical_range':
// '[-8192|8191]', 'bit': 29, 'type': 'int', 'order': 'motorola',
// 'physical_unit': 'm'}
int Vehicle14f0::can_rx_radius_curvature(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 18;
  x >>= 18;

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_rx_yaw_rate_validity', 'enum': {0:
// 'CAN_RX_YAW_RATE_VALIDITY_INVALID', 1: 'CAN_RX_YAW_RATE_VALIDITY_VALID'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 31, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Vehicle1_4f0::Can_rx_yaw_rate_validityType
Vehicle14f0::can_rx_yaw_rate_validity(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(7, 1);

  Vehicle1_4f0::Can_rx_yaw_rate_validityType ret =
      static_cast<Vehicle1_4f0::Can_rx_yaw_rate_validityType>(x);
  return ret;
}

// config detail: {'name': 'can_rx_yaw_rate', 'offset': 0.0, 'precision':
// 0.0625, 'len': 12, 'is_signed_var': True, 'physical_range':
// '[-128|127.9375]', 'bit': 11, 'type': 'double', 'order': 'motorola',
// 'physical_unit': 'deg/s'}
double Vehicle14f0::can_rx_yaw_rate(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.062500;
  return ret;
}

// config detail: {'name': 'can_rx_vehicle_speed_direction', 'enum': {0:
// 'CAN_RX_VEHICLE_SPEED_DIRECTION_FORWARD', 1:
// 'CAN_RX_VEHICLE_SPEED_DIRECTION_REVERSE'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 12,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Vehicle1_4f0::Can_rx_vehicle_speed_directionType
Vehicle14f0::can_rx_vehicle_speed_direction(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  Vehicle1_4f0::Can_rx_vehicle_speed_directionType ret =
      static_cast<Vehicle1_4f0::Can_rx_vehicle_speed_directionType>(x);
  return ret;
}

// config detail: {'name': 'can_rx_vehicle_speed', 'offset': 0.0, 'precision':
// 0.0625, 'len': 11, 'is_signed_var': False, 'physical_range': '[0|127.9375]',
// 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
double Vehicle14f0::can_rx_vehicle_speed(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(5, 3);
  x <<= 3;
  x |= t;

  double ret = x * 0.062500;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
