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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

class Vcubrakereport47 : public ::apollo::drivers::canbus::ProtocolData<
                             ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcubrakereport47();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'Brake_Enable_Resp', 'is_signed_var':
  // False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': ''}
  bool brake_enable_resp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Standby;0x1:auto drive;0x2:net
  // drive;0x3:remote control;0x4:emergency brake;0x5~0x7:Reserved', 'enum': {0:
  // 'CONTROL_MODE_RESP_STANDBY', 1: 'CONTROL_MODE_RESP_AUTO_DRIVE', 2:
  // 'CONTROL_MODE_RESP_NET_DRIVE', 3: 'CONTROL_MODE_RESP_REMOTE_CONTROL', 4:
  // 'CONTROL_MODE_RESP_EMERGENCY_BRAKE'}, 'precision': 1.0, 'len': 3, 'name':
  // 'Control_Mode_Resp', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|7]', 'bit': 6, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': 'bit'}
  Vcu_brake_report_47::Control_mode_respType control_mode_resp(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'VCU_Real_Brake_Valid',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool vcu_real_brake_valid(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'VCU_Real_Brake', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 15,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int vcu_real_brake(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description':
  // '0x0:EPB_Released;0x1:EPB_Applied;0x2:EPB_Releasing;0x3:EPB_Fault;0x4:EPB_Applying',
  // 'offset': 0.0, 'precision': 1.0, 'len': 3, 'name':
  // 'VCU_Real_Parking_Status', 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 18, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int vcu_real_parking_status(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'VCU_Real_Parking_Valid',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 19, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool vcu_real_parking_valid(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'description': '0x0:off;0x1:on', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'RampAuxiliaryIndication',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 20, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool rampauxiliaryindication(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'name': 'VehicleSlope', 'offset': 0.0, 'precision':
  // 0.1758125, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|45]',
  // 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit':
  // '\xc2\xb0'}
  double vehicleslope(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_BrakeRept_AliveCounter', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int vcu_brakerept_alivecounter(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'name': 'VCU_BrakeRept_CheckSum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int vcu_brakerept_checksum(const std::uint8_t* bytes,
                             const int32_t length) const;
};

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
