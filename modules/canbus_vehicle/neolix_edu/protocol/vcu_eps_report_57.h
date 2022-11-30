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

class Vcuepsreport57 : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::Neolix_edu> {
 public:
  static const int32_t ID;
  Vcuepsreport57();
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
  Vcu_eps_report_57::Control_mode_respType control_mode_resp(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:not overflow;0x1:overflow', 'offset':
  // 0.0, 'precision': 1.0, 'len': 1, 'name': 'VCU_EPS_Report', 'is_signed_var':
  // False, 'physical_range': '[0|0]', 'bit': 7, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': ''}
  bool vcu_eps_report(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': ';', 'offset': -2048.0, 'precision': 0.0625,
  // 'len': 16, 'name': 'VCU_Real_Angle', 'is_signed_var': False,
  // 'physical_range': '[0|0]', 'bit': 23, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': ''}
  double vcu_real_angle(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'VCU_Real_Angle_Valid',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 32, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool vcu_real_angle_valid(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'description': '0x0:disable;0x1:enable;', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'VCU_Target_Angle_Valid',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 33, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool vcu_target_angle_valid(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'VCU_Target_Angle', 'offset': -512.0, 'precision':
  // 0.25, 'len': 12, 'is_signed_var': False, 'physical_range': '[-380|380]',
  // 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
  double vcu_target_angle(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'name': 'VCU_EPS_Rept_AliveCounter', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int vcu_eps_rept_alivecounter(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'name': 'VCU_EPS_Rept_CheckSum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int vcu_eps_rept_checksum(const std::uint8_t* bytes,
                            const int32_t length) const;
};

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
