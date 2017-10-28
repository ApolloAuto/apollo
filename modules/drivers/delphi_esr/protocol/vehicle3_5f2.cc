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

#include "modules/drivers/delphi_esr/protocol/vehicle3_5f2.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Vehicle35f2::Vehicle35f2() {}
const int32_t Vehicle35f2::ID = 0x5F2;

void Vehicle35f2::Parse(const std::uint8_t* bytes, int32_t length,
                        DelphiESR* delphi_esr) const {
  delphi_esr->mutable_vehicle3_5f2()->set_can_rx_serv_align_updates_need(
      can_rx_serv_align_updates_need(bytes, length));
  delphi_esr->mutable_vehicle3_5f2()->set_can_rx_serv_align_type(
      can_rx_serv_align_type(bytes, length));
  delphi_esr->mutable_vehicle3_5f2()->set_can_rx_serv_align_enable(
      can_rx_serv_align_enable(bytes, length));
  delphi_esr->mutable_vehicle3_5f2()->set_can_rx_aalign_avg_ctr_total(
      can_rx_aalign_avg_ctr_total(bytes, length));
  delphi_esr->mutable_vehicle3_5f2()->set_can_rx_auto_align_converged(
      can_rx_auto_align_converged(bytes, length));
  delphi_esr->mutable_vehicle3_5f2()->set_can_rx_auto_align_disable(
      can_rx_auto_align_disable(bytes, length));
  delphi_esr->mutable_vehicle3_5f2()->set_can_rx_angle_mounting_offset(
      can_rx_angle_mounting_offset(bytes, length));
  delphi_esr->mutable_vehicle3_5f2()->set_can_rx_wheel_slip(
      can_rx_wheel_slip(bytes, length));
  delphi_esr->mutable_vehicle3_5f2()->set_can_rx_radar_height(
      can_rx_radar_height(bytes, length));
  delphi_esr->mutable_vehicle3_5f2()->set_can_rx_radar_fov_mr(
      can_rx_radar_fov_mr(bytes, length));
  delphi_esr->mutable_vehicle3_5f2()->set_can_rx_radar_fov_lr(
      can_rx_radar_fov_lr(bytes, length));
  delphi_esr->mutable_vehicle3_5f2()->set_can_rx_long_accel_validity(
      can_rx_long_accel_validity(bytes, length));
  delphi_esr->mutable_vehicle3_5f2()->set_can_rx_long_accel(
      can_rx_long_accel(bytes, length));
  delphi_esr->mutable_vehicle3_5f2()->set_can_rx_lat_accel_validity(
      can_rx_lat_accel_validity(bytes, length));
  delphi_esr->mutable_vehicle3_5f2()->set_can_rx_lat_accel(
      can_rx_lat_accel(bytes, length));
}

// config detail: {'name': 'can_rx_serv_align_updates_need', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vehicle35f2::can_rx_serv_align_updates_need(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_rx_serv_align_type', 'enum': {0:
// 'CAN_RX_SERV_ALIGN_TYPE_AUTO_OR_DEALER', 1:
// 'CAN_RX_SERV_ALIGN_TYPE_VOLVO_SHORT_TRACK'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 47,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Vehicle3_5f2::Can_rx_serv_align_typeType Vehicle35f2::can_rx_serv_align_type(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(7, 1);

  Vehicle3_5f2::Can_rx_serv_align_typeType ret =
      static_cast<Vehicle3_5f2::Can_rx_serv_align_typeType>(x);
  return ret;
}

// config detail: {'name': 'can_rx_serv_align_enable', 'enum': {0:
// 'CAN_RX_SERV_ALIGN_ENABLE_DISABLED', 1: 'CAN_RX_SERV_ALIGN_ENABLE_ENABLED'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 46, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Vehicle3_5f2::Can_rx_serv_align_enableType
Vehicle35f2::can_rx_serv_align_enable(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(6, 1);

  Vehicle3_5f2::Can_rx_serv_align_enableType ret =
      static_cast<Vehicle3_5f2::Can_rx_serv_align_enableType>(x);
  return ret;
}

// config detail: {'name': 'can_rx_aalign_avg_ctr_total', 'offset': 250.0,
// 'precision': 250.0, 'len': 3, 'is_signed_var': False, 'physical_range':
// '[250|2000]', 'bit': 45, 'type': 'double', 'order': 'motorola',
// 'physical_unit': ''}
double Vehicle35f2::can_rx_aalign_avg_ctr_total(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(3, 3);

  double ret = x * 250.000000 + 250.000000;
  return ret;
}

// config detail: {'name': 'can_rx_auto_align_converged', 'enum': {0:
// 'CAN_RX_AUTO_ALIGN_CONVERGED_NOT_CONVERGED', 1:
// 'CAN_RX_AUTO_ALIGN_CONVERGED_CONVERGED'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 42,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Vehicle3_5f2::Can_rx_auto_align_convergedType
Vehicle35f2::can_rx_auto_align_converged(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(2, 1);

  Vehicle3_5f2::Can_rx_auto_align_convergedType ret =
      static_cast<Vehicle3_5f2::Can_rx_auto_align_convergedType>(x);
  return ret;
}

// config detail: {'name': 'can_rx_auto_align_disable', 'enum': {0:
// 'CAN_RX_AUTO_ALIGN_DISABLE_ENABLED', 1:
// 'CAN_RX_AUTO_ALIGN_DISABLE_DISABLED'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 39,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Vehicle3_5f2::Can_rx_auto_align_disableType
Vehicle35f2::can_rx_auto_align_disable(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(7, 1);

  Vehicle3_5f2::Can_rx_auto_align_disableType ret =
      static_cast<Vehicle3_5f2::Can_rx_auto_align_disableType>(x);
  return ret;
}

// config detail: {'description': '(+) = clockwise', 'offset': 0.0, 'precision':
// 0.0625, 'len': 8, 'name': 'can_rx_angle_mounting_offset', 'is_signed_var':
// True, 'physical_range': '[-8|7.9375]', 'bit': 63, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'deg'}
double Vehicle35f2::can_rx_angle_mounting_offset(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.062500;
  return ret;
}

// config detail: {'name': 'can_rx_wheel_slip', 'enum': {0:
// 'CAN_RX_WHEEL_SLIP_NO_CONTROL', 1: 'CAN_RX_WHEEL_SLIP_BRAKE_SLIP_CONTROL', 2:
// 'CAN_RX_WHEEL_SLIP_TRACTION_SLIP_CONTROL', 3: 'CAN_RX_WHEEL_SLIP_INVALID_3'},
// 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 41, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Vehicle3_5f2::Can_rx_wheel_slipType Vehicle35f2::can_rx_wheel_slip(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 2);

  Vehicle3_5f2::Can_rx_wheel_slipType ret =
      static_cast<Vehicle3_5f2::Can_rx_wheel_slipType>(x);
  return ret;
}

// config detail: {'name': 'can_rx_radar_height', 'offset': 0.0, 'precision':
// 1.0, 'len': 7, 'is_signed_var': False, 'physical_range': '[0|125]', 'bit':
// 38, 'type': 'int', 'order': 'motorola', 'physical_unit': 'cm'}
int Vehicle35f2::can_rx_radar_height(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 7);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_rx_radar_fov_mr', 'offset': 0.0, 'precision':
// 1.0, 'len': 7, 'is_signed_var': False, 'physical_range': '[0|120]', 'bit':
// 30, 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg'}
int Vehicle35f2::can_rx_radar_fov_mr(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 7);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_rx_radar_fov_lr', 'offset': 0.0, 'precision':
// 1.0, 'len': 5, 'is_signed_var': False, 'physical_range': '[0|30]', 'bit': 19,
// 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg'}
int Vehicle35f2::can_rx_radar_fov_lr(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(7, 1);
  x <<= 1;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_rx_long_accel_validity', 'enum': {0:
// 'CAN_RX_LONG_ACCEL_VALIDITY_INVALID', 1: 'CAN_RX_LONG_ACCEL_VALIDITY_VALID'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Vehicle3_5f2::Can_rx_long_accel_validityType
Vehicle35f2::can_rx_long_accel_validity(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  Vehicle3_5f2::Can_rx_long_accel_validityType ret =
      static_cast<Vehicle3_5f2::Can_rx_long_accel_validityType>(x);
  return ret;
}

// config detail: {'name': 'can_rx_long_accel', 'offset': 0.0, 'precision':
// 0.03125, 'len': 9, 'is_signed_var': True, 'physical_range': '[-8|7.96875]',
// 'bit': 12, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s/s'}
double Vehicle35f2::can_rx_long_accel(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 5);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 23;
  x >>= 23;

  double ret = x * 0.031250;
  return ret;
}

// config detail: {'name': 'can_rx_lat_accel_validity', 'enum': {0:
// 'CAN_RX_LAT_ACCEL_VALIDITY_INVALID', 1: 'CAN_RX_LAT_ACCEL_VALIDITY_VALID'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 6, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Vehicle3_5f2::Can_rx_lat_accel_validityType
Vehicle35f2::can_rx_lat_accel_validity(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 1);

  Vehicle3_5f2::Can_rx_lat_accel_validityType ret =
      static_cast<Vehicle3_5f2::Can_rx_lat_accel_validityType>(x);
  return ret;
}

// config detail: {'name': 'can_rx_lat_accel', 'offset': 0.0, 'precision':
// 0.03125, 'len': 9, 'is_signed_var': True, 'physical_range': '[-8|7.96875]',
// 'bit': 5, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Vehicle35f2::can_rx_lat_accel(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(5, 3);
  x <<= 3;
  x |= t;

  x <<= 23;
  x >>= 23;

  double ret = x * 0.031250;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
