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

#include "modules/canbus_vehicle/ch/proto/ch.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ch {

class Ecustatus1515 : public ::apollo::drivers::canbus::ProtocolData<
                          ::apollo::canbus::Ch> {
 public:
  static const int32_t ID;
  Ecustatus1515();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ch* chassis) const override;

 private:
  // config detail: {'bit': 0, 'description': 'Current speed (Steering status)',
  // 'is_signed_var': True, 'len': 16, 'name': 'SPEED', 'offset': 0.0, 'order':
  // 'intel', 'physical_range': '[-327.68|327.67]', 'physical_unit': 'm/s',
  // 'precision': 0.01, 'type': 'double'}
  double speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'description': 'Current acceleration (Steering
  // status)', 'is_signed_var': True, 'len': 16, 'name': 'ACC_SPEED', 'offset':
  // 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'm/s^2',
  // 'precision': 0.001, 'type': 'double'}
  double acc_speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'description': 'Current Auto-mode state (Chassis
  // status)', 'enum': {0: 'CTRL_STS_OUT_OF_CONTROL', 1:
  // 'CTRL_STS_UNDER_CONTROL'}, 'is_signed_var': False, 'len': 8, 'name':
  // 'CTRL_STS', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Ecu_status_1_515::Ctrl_stsType ctrl_sts(const std::uint8_t* bytes,
                                          const int32_t length) const;

  // config detail: {'bit': 40, 'description': 'Current chassis state (Chassis
  // status)', 'is_signed_var': False, 'len': 8, 'name': 'CHASSIS_STS',
  // 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassis_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 48, 'description': 'Chassis error code (Chassis
  // status)', 'is_signed_var': False, 'len': 16, 'name': 'CHASSIS_ERR',
  // 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassis_err(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 48, 'description': 'Chassis error code (Chassis
  // status)', 'enum': {0: 'CHASSIS_ADS_ERR_NOMAL', 1:
  // 'CHASSIS_ADS_ERR_ADS_CAN_LOST', 2: 'CHASSIS_ADS_ERR_ADS_CAN_RECOVERY'},
  // 'is_signed_var': False, 'len': 2, 'name': 'CHASSIS_ADS_ERR', 'offset': 0.0,
  // 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  Ecu_status_1_515::Chassis_ads_errType chassis_ads_err(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 50, 'enum': {0: 'CHASSIS_BMS_CAN_NORMAL', 1:
  // 'CHASSIS_BMS_CAN_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'CHASSIS_BMS_CAN', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Ecu_status_1_515::Chassis_bms_canType chassis_bms_can(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 51, 'enum': {0: 'CHASSIS_EHB_CAN_NORMAL', 1:
  // 'CHASSIS_EHB_CAN_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'CHASSIS_EHB_CAN', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Ecu_status_1_515::Chassis_ehb_canType chassis_ehb_can(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 52, 'enum': {0: 'CHASSIS_EHB_ERR_NORMAL', 1:
  // 'CHASSIS_EHB_ERR_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'CHASSIS_EHB_ERR', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Ecu_status_1_515::Chassis_ehb_errType chassis_ehb_err(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 53, 'enum': {0: 'CHASSIS_EPS_CAN_NORMAL', 1:
  // 'CHASSIS_EPS_CAN_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'CHASSIS_EPS_CAN', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Ecu_status_1_515::Chassis_eps_canType chassis_eps_can(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 54, 'enum': {0: 'CHASSIS_EPS_ERR_NORMAL', 1:
  // 'CHASSIS_EPS_ERR_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'CHASSIS_EPS_ERR', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Ecu_status_1_515::Chassis_eps_errType chassis_eps_err(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 55, 'enum': {0: 'CHASSIS_HW_LOST_NORMAL', 1:
  // 'CHASSIS_HW_LOST_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'CHASSIS_HW_Lost', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Ecu_status_1_515::Chassis_hw_lostType chassis_hw_lost(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 56, 'enum': {0: 'CHASSIS_MCU_CAN_NORMAL', 1:
  // 'CHASSIS_MCU_CAN_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'CHASSIS_MCU_CAN', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Ecu_status_1_515::Chassis_mcu_canType chassis_mcu_can(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 57, 'enum': {0: 'CHASSIS_MCU_ERR_NORMAL', 1:
  // 'CHASSIS_MCU_ERR_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'CHASSIS_MCU_ERR', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Ecu_status_1_515::Chassis_mcu_errType chassis_mcu_err(
      const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
