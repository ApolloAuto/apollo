/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
namespace minibus {

class Vcubasicmessage18ffea97 : public ::apollo::drivers::canbus::ProtocolData<
                                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcubasicmessage18ffea97();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 62, 'is_signed_var': False, 'len': 1, 'name':
  // 'VCU_Basic_Onebit', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool vcu_basic_onebit(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 61, 'is_signed_var': False, 'len': 1, 'name':
  // 'VCU_Basic_HP_Halt', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool vcu_basic_hp_halt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 59, 'is_signed_var': True, 'len': 2, 'name':
  // 'VCU_Basic_GeatDefault_Code', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int vcu_basic_geatdefault_code(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 3, 'name':
  // 'VCU_Basic_SysDefault_Level', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int vcu_basic_sysdefault_level(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'bit': 48, 'is_signed_var': True, 'len': 8, 'name':
  // 'VCU_Basic_MotoController_Tempreture', 'offset': -40.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int vcu_basic_motocontroller_tempreture(const std::uint8_t* bytes,
                                          const int32_t length) const;

  // config detail: {'bit': 40, 'is_signed_var': True, 'len': 8, 'name':
  // 'VCU_Basic_Motor_Tempreture', 'offset': -40.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int vcu_basic_motor_tempreture(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'bit': 37, 'is_signed_var': False, 'len': 3, 'name':
  // 'VCU_Basic_Charge_Status', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int vcu_basic_charge_status(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 34, 'is_signed_var': False, 'len': 3, 'name':
  // 'VCU_Basic_Real_HighVoltage_Sta', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int vcu_basic_real_highvoltage_sta(const std::uint8_t* bytes,
                                     const int32_t length) const;

  // config detail: {'bit': 32, 'enum': {0: 'VCU_BASIC_REAL_GEAR_INVALID', 1:
  // 'VCU_BASIC_REAL_GEAR_R', 2: 'VCU_BASIC_REAL_GEAR_N', 3:
  // 'VCU_BASIC_REAL_GEAR_D'}, 'is_signed_var': False, 'len': 2, 'name':
  // 'VCU_Basic_Real_Gear', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Vcu_basic_message_18ffea97::Vcu_basic_real_gearType vcu_basic_real_gear(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name':
  // 'VCU_Basic_Motor_Speed', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': 'rpm', 'precision': 0.5, 'type': 'double'}
  double vcu_basic_motor_speed(const std::uint8_t* bytes,
                               const int32_t length) const;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
