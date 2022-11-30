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

class Vcuvehiclestatusreport101
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::Neolix_edu> {
 public:
  static const int32_t ID;
  Vcuvehiclestatusreport101();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Neolix_edu* chassis) const override;

 private:
  // config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'Drive_Enable_Resp', 'is_signed_var':
  // False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': ''}
  bool drive_enable_resp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Disconnect;0x1:Connect', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'VCU_HighVoltageCircuitState',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 2, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool vcu_highvoltagecircuitstate(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'description': '0x0: Disable;0x1:Enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'VCU_DCDC_EnabledStates',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 3, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool vcu_dcdc_enabledstates(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'description': '0x0:Standby;0x1:auto drive;0x2:net
  // drive;0x3:remote control;0x4:emergency brake;0x5~0x7:Reserved', 'enum': {0:
  // 'CONTROL_MODE_RESP_STANDBY', 1: 'CONTROL_MODE_RESP_AUTO_DRIVE', 2:
  // 'CONTROL_MODE_RESP_NET_DRIVE', 3: 'CONTROL_MODE_RESP_REMOTE_CONTROL', 4:
  // 'CONTROL_MODE_RESP_EMERGENCY_BRAKE'}, 'precision': 1.0, 'len': 3, 'name':
  // 'Control_Mode_Resp', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|7]', 'bit': 6, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Vcu_vehicle_status_report_101::Control_mode_respType control_mode_resp(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_Vehicle_Speed', 'offset': 0.0, 'precision':
  // 0.05625, 'len': 13, 'is_signed_var': False, 'physical_range': '[0|460.69]',
  // 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Km/h'}
  double vcu_vehicle_speed(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'description': '0x0:Reserved;0x1:Start;0x2:Stop;0x3:Invalid
  // ', 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name':
  // 'VCU_LowBatteryChargingFunctionSt', 'is_signed_var': False,
  // 'physical_range': '[0|0]', 'bit': 17, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': ''}
  int vcu_lowbatterychargingfunctionst(const std::uint8_t* bytes,
                                       const int32_t length) const;

  // config detail: {'name': 'VCU_Display_SOC', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 31,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': '%'}
  int vcu_display_soc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_Motor_Speed', 'offset': 0.0, 'precision':
  // 0.25, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
  // 39, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  double vcu_motor_speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Standby Status;0x1:Forward
  // Mode;0x2:Reverse Mode', 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name':
  // 'VCU_Motor_Direction', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 54, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int vcu_motor_direction(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'VCU_Motor_Speed_Valid',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 55, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool vcu_motor_speed_valid(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'name': 'VCU_StatusRept_AliveCounter', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int vcu_statusrept_alivecounter(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'name': 'VCU_StatusRept_CheckSum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int vcu_statusrept_checksum(const std::uint8_t* bytes,
                              const int32_t length) const;
};

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
