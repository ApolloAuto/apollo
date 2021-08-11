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

class Vcuvehiclefaultresponse201
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcuvehiclefaultresponse201();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': '0x0: no error;0x1: level 1 error;0x2: level
  // 2 error;0x3: level 3 error;0x4: level 4 error;0x5: level 5 error',
  // 'offset': 0.0, 'precision': 1.0, 'len': 4, 'name':
  // 'Vehicle_Error_IndicationsVCU', 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 3, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int vehicle_error_indicationsvcu(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'description': '0x0: no error;0x1: level 1 error;0x2: level
  // 2 error;0x3: level 3 error;0x4: level 4 error;0x5: level 5 error',
  // 'offset': 0.0, 'precision': 1.0, 'len': 4, 'name': 'Brake_System_ErrorEHB',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7, 'type': 'int',
  // 'order': 'motorola', 'physical_unit': ''}
  int brake_system_errorehb(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'description': '0x0: no error;0x1: level 1 error;0x2: level
  // 2 error;0x3: level 3 error;0x4: level 4 error;0x5: level 5 error',
  // 'offset': 0.0, 'precision': 1.0, 'len': 4, 'name': 'EPS_Error',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 11, 'type':
  // 'int', 'order': 'motorola', 'physical_unit': ''}
  int eps_error(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0: no error;0x1: level 1 error;0x2: level
  // 2 error;0x3: level 3 error;0x4: level 4 error;0x5: level 5 error',
  // 'offset': 0.0, 'precision': 1.0, 'len': 4, 'name': 'Motor_Error',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 15, 'type':
  // 'int', 'order': 'motorola', 'physical_unit': ''}
  int motor_error(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0: no error;0x1: level 1 error;0x2: level
  // 2 error;0x3: level 3 error;0x4: level 4 error;0x5: level 5 error',
  // 'offset': 0.0, 'precision': 1.0, 'len': 4, 'name': 'EPB_Error',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 19, 'type':
  // 'int', 'order': 'motorola', 'physical_unit': ''}
  int epb_error(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0: no error;0x1: level 1 error;0x2: level
  // 2 error;0x3: level 3 error;0x4: level 4 error;0x5: level 5 error',
  // 'offset': 0.0, 'precision': 1.0, 'len': 4, 'name':
  // 'High_Voltage_Battery_ErrorBCU', 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int high_voltage_battery_errorbcu(const std::uint8_t* bytes,
                                    const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'AutoMode_Exit_Reason_LossCommuni',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 32, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool automode_exit_reason_losscommuni(const std::uint8_t* bytes,
                                        const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'AutoMode_Exit_Reason_ReqSignalNo',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 33, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool automode_exit_reason_reqsignalno(const std::uint8_t* bytes,
                                        const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'AutoMode_Exit_Reason_Low_Power',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 34, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool automode_exit_reason_low_power(const std::uint8_t* bytes,
                                      const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'AutoMode_Exit_Reason_HighVolt',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 35, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool automode_exit_reason_highvolt(const std::uint8_t* bytes,
                                     const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'AutoMode_Exit_Reason_Vehicle_Flt',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 36, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool automode_exit_reason_vehicle_flt(const std::uint8_t* bytes,
                                        const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'AutoMode_Exit_Reason_Press_Emerg',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 37, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool automode_exit_reason_press_emerg(const std::uint8_t* bytes,
                                        const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'AutoMode_Exit_Reason_Press_Remot',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 38, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool automode_exit_reason_press_remot(const std::uint8_t* bytes,
                                        const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'AutoMode_Exit_Reason_PDU_Control',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool automode_exit_reason_pdu_control(const std::uint8_t* bytes,
                                        const int32_t length) const;

  // config detail: {'name': 'VCU_FaultRept_AliveCounter', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int vcu_faultrept_alivecounter(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'name': 'VCU_FaultRept_CheckSum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int vcu_faultrept_checksum(const std::uint8_t* bytes,
                             const int32_t length) const;
};

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
