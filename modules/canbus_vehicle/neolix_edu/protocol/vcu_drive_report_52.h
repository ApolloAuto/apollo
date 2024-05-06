/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/neolix_edu/proto/neolix_edu.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

class Vcudrivereport52 : public ::apollo::drivers::canbus::ProtocolData<
                             ::apollo::canbus::Neolix_edu> {
 public:
  static const int32_t ID;
  Vcudrivereport52();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Neolix_edu* chassis) const override;

 private:
  // config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'Drive_Enable_Resp', 'is_signed_var':
  // False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': ''}
  bool drive_enable_resp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Standby;0x1:auto drive;0x2:net
  // drive;0x3:remote control;0x4:emergency brake;0x5~0x7:Reserved', 'enum': {0:
  // 'CONTROL_MODE_RESP_STANDBY', 1: 'CONTROL_MODE_RESP_AUTO_DRIVE', 2:
  // 'CONTROL_MODE_RESP_NET_DRIVE', 3: 'CONTROL_MODE_RESP_REMOTE_CONTROL', 4:
  // 'CONTROL_MODE_RESP_EMERGENCY_BRAKE'}, 'precision': 1.0, 'len': 3, 'name':
  // 'Control_Mode_Resp', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|7]', 'bit': 6, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Vcu_drive_report_52::Control_mode_respType control_mode_resp(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description':
  // '0x0:N\xe6\xa1\xa3;0x1:D\xe6\xa1\xa3;0x2:R\xe6\xa1\xa3;0x3:Reserved',
  // 'enum': {0: 'VCU_REAL_SHIFT_N', 1: 'VCU_REAL_SHIFT_D', 2:
  // 'VCU_REAL_SHIFT_R', 3: 'VCU_REAL_SHIFT_RESERVED'}, 'precision': 1.0, 'len':
  // 2, 'name': 'VCU_Real_Shift', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|3]', 'bit': 9, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Vcu_drive_report_52::Vcu_real_shiftType vcu_real_shift(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'VCU_Real_Shift_Valid',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 10, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool vcu_real_shift_valid(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'VCU_Real_Torque_Valid',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 11, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool vcu_real_torque_valid(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'name': 'VCU_Real_Torque', 'offset': -665.0, 'precision':
  // 0.02, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
  // 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Nm'}
  double vcu_real_torque(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'VCU_LimitedTorqueMode',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 32, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool vcu_limitedtorquemode(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'name': 'VCU_DriveRept_AliveCounter', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int vcu_driverept_alivecounter(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'name': 'VCU_DriveRept_CheckSum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int vcu_driverept_checksum(const std::uint8_t* bytes,
                             const int32_t length) const;
};

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
