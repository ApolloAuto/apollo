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

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_VEHICLE3_5F2_H_
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_VEHICLE3_5F2_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::DelphiESR;

class Vehicle35f2 : public apollo::drivers::canbus::ProtocolData<DelphiESR> {
 public:
  static const int32_t ID;
  Vehicle35f2();
  void Parse(const std::uint8_t* bytes, int32_t length,
             DelphiESR* delphi_esr) const override;

 private:
  // config detail: {'name': 'CAN_RX_SERV_ALIGN_UPDATES_NEED', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_rx_serv_align_updates_need(const std::uint8_t* bytes,
                                     const int32_t length) const;

  // config detail: {'name': 'CAN_RX_SERV_ALIGN_TYPE', 'enum': {0:
  // 'CAN_RX_SERV_ALIGN_TYPE_AUTO_OR_DEALER', 1:
  // 'CAN_RX_SERV_ALIGN_TYPE_VOLVO_SHORT_TRACK'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle3_5f2::Can_rx_serv_align_typeType can_rx_serv_align_type(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_SERV_ALIGN_ENABLE', 'enum': {0:
  // 'CAN_RX_SERV_ALIGN_ENABLE_DISABLED', 1:
  // 'CAN_RX_SERV_ALIGN_ENABLE_ENABLED'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 46, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle3_5f2::Can_rx_serv_align_enableType can_rx_serv_align_enable(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_AALIGN_AVG_CTR_TOTAL', 'offset': 250.0,
  // 'precision': 250.0, 'len': 3, 'is_signed_var': False, 'physical_range':
  // '[250|2000]', 'bit': 45, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': ''}
  double can_rx_aalign_avg_ctr_total(const std::uint8_t* bytes,
                                     const int32_t length) const;

  // config detail: {'name': 'CAN_RX_AUTO_ALIGN_CONVERGED', 'enum': {0:
  // 'CAN_RX_AUTO_ALIGN_CONVERGED_NOT_CONVERGED', 1:
  // 'CAN_RX_AUTO_ALIGN_CONVERGED_CONVERGED'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 42, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle3_5f2::Can_rx_auto_align_convergedType can_rx_auto_align_converged(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_AUTO_ALIGN_DISABLE', 'enum': {0:
  // 'CAN_RX_AUTO_ALIGN_DISABLE_ENABLED', 1:
  // 'CAN_RX_AUTO_ALIGN_DISABLE_DISABLED'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle3_5f2::Can_rx_auto_align_disableType can_rx_auto_align_disable(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '(+) = clockwise', 'offset': 0.0,
  // 'precision': 0.0625, 'len': 8, 'name': 'CAN_RX_ANGLE_MOUNTING_OFFSET',
  // 'is_signed_var': True, 'physical_range': '[-8|7.9375]', 'bit': 63, 'type':
  // 'double', 'order': 'motorola', 'physical_unit': 'deg'}
  double can_rx_angle_mounting_offset(const std::uint8_t* bytes,
                                      const int32_t length) const;

  // config detail: {'name': 'CAN_RX_WHEEL_SLIP', 'enum': {0:
  // 'CAN_RX_WHEEL_SLIP_NO_CONTROL', 1: 'CAN_RX_WHEEL_SLIP_BRAKE_SLIP_CONTROL',
  // 2: 'CAN_RX_WHEEL_SLIP_TRACTION_SLIP_CONTROL', 3:
  // 'CAN_RX_WHEEL_SLIP_INVALID_3'}, 'precision': 1.0, 'len': 2,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 41, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle3_5f2::Can_rx_wheel_slipType can_rx_wheel_slip(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_RADAR_HEIGHT', 'offset': 0.0, 'precision':
  // 1.0, 'len': 7, 'is_signed_var': False, 'physical_range': '[0|125]', 'bit':
  // 38, 'type': 'int', 'order': 'motorola', 'physical_unit': 'cm'}
  int can_rx_radar_height(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'name': 'CAN_RX_RADAR_FOV_MR', 'offset': 0.0, 'precision':
  // 1.0, 'len': 7, 'is_signed_var': False, 'physical_range': '[0|120]', 'bit':
  // 30, 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg'}
  int can_rx_radar_fov_mr(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'name': 'CAN_RX_RADAR_FOV_LR', 'offset': 0.0, 'precision':
  // 1.0, 'len': 5, 'is_signed_var': False, 'physical_range': '[0|30]', 'bit':
  // 19, 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg'}
  int can_rx_radar_fov_lr(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'name': 'CAN_RX_LONG_ACCEL_VALIDITY', 'enum': {0:
  // 'CAN_RX_LONG_ACCEL_VALIDITY_INVALID', 1:
  // 'CAN_RX_LONG_ACCEL_VALIDITY_VALID'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 7,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle3_5f2::Can_rx_long_accel_validityType can_rx_long_accel_validity(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_LONG_ACCEL', 'offset': 0.0, 'precision':
  // 0.03125, 'len': 9, 'is_signed_var': True, 'physical_range': '[-8|7.96875]',
  // 'bit': 12, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s/s'}
  double can_rx_long_accel(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'name': 'CAN_RX_LAT_ACCEL_VALIDITY', 'enum': {0:
  // 'CAN_RX_LAT_ACCEL_VALIDITY_INVALID', 1: 'CAN_RX_LAT_ACCEL_VALIDITY_VALID'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 6, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Vehicle3_5f2::Can_rx_lat_accel_validityType can_rx_lat_accel_validity(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_LAT_ACCEL', 'offset': 0.0, 'precision':
  // 0.03125, 'len': 9, 'is_signed_var': True, 'physical_range': '[-8|7.96875]',
  // 'bit': 5, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  double can_rx_lat_accel(const std::uint8_t* bytes,
                          const int32_t length) const;
};

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_VEHICLE3_5F2_H_
