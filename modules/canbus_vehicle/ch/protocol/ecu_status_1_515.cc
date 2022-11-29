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

#include "modules/canbus_vehicle/ch/protocol/ecu_status_1_515.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Ecustatus1515::Ecustatus1515() {}
const int32_t Ecustatus1515::ID = 0x515;

void Ecustatus1515::Parse(const std::uint8_t* bytes, int32_t length,
                          Ch* chassis) const {
  chassis->mutable_ecu_status_1_515()->set_chassis_mcu_err(
      chassis_mcu_err(bytes, length));
  chassis->mutable_ecu_status_1_515()->set_chassis_mcu_can(
      chassis_mcu_can(bytes, length));
  chassis->mutable_ecu_status_1_515()->set_chassis_hw_lost(
      chassis_hw_lost(bytes, length));
  chassis->mutable_ecu_status_1_515()->set_chassis_eps_err(
      chassis_eps_err(bytes, length));
  chassis->mutable_ecu_status_1_515()->set_chassis_eps_can(
      chassis_eps_can(bytes, length));
  chassis->mutable_ecu_status_1_515()->set_chassis_ehb_err(
      chassis_ehb_err(bytes, length));
  chassis->mutable_ecu_status_1_515()->set_chassis_ehb_can(
      chassis_ehb_can(bytes, length));
  chassis->mutable_ecu_status_1_515()->set_chassis_bms_can(
      chassis_bms_can(bytes, length));
  chassis->mutable_ecu_status_1_515()->set_speed(
      speed(bytes, length));
  chassis->mutable_ecu_status_1_515()->set_acc_speed(
      acc_speed(bytes, length));
  chassis->mutable_ecu_status_1_515()->set_ctrl_sts(
      ctrl_sts(bytes, length));
  chassis->mutable_ecu_status_1_515()->set_chassis_sts(
      chassis_sts(bytes, length));
  chassis->mutable_ecu_status_1_515()->set_chassis_err(
      chassis_err(bytes, length));
  chassis->mutable_ecu_status_1_515()->set_chassis_ads_err(
      chassis_ads_err(bytes, length));
}

// config detail: {'bit': 0, 'description': 'Current speed (Steering status)',
// 'is_signed_var': True, 'len': 16, 'name': 'speed', 'offset': 0.0, 'order':
// 'intel', 'physical_range': '[-327.68|327.67]', 'physical_unit': 'm/s',
// 'precision': 0.01, 'type': 'double'}
double Ecustatus1515::speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 16, 'description': 'Current acceleration (Steering
// status)', 'is_signed_var': True, 'len': 16, 'name': 'acc_speed', 'offset':
// 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'm/s^2',
// 'precision': 0.001, 'type': 'double'}
double Ecustatus1515::acc_speed(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'bit': 32, 'description': 'Current Auto-mode state (Chassis
// status)', 'enum': {0: 'CTRL_STS_OUT_OF_CONTROL', 1:
// 'CTRL_STS_UNDER_CONTROL'}, 'is_signed_var': False, 'len': 8, 'name':
// 'ctrl_sts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Ecu_status_1_515::Ctrl_stsType Ecustatus1515::ctrl_sts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Ecu_status_1_515::Ctrl_stsType ret =
      static_cast<Ecu_status_1_515::Ctrl_stsType>(x);
  return ret;
}

// config detail: {'bit': 40, 'description': 'Current chassis state (Chassis
// status)', 'is_signed_var': False, 'len': 8, 'name': 'chassis_sts', 'offset':
// 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'int'}
int Ecustatus1515::chassis_sts(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 48, 'description': 'Chassis error code (Chassis
// status)', 'is_signed_var': False, 'len': 16, 'name': 'chassis_err', 'offset':
// 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'int'}
int Ecustatus1515::chassis_err(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 48, 'description': 'Chassis error code (Chassis
// status)', 'enum': {0: 'CHASSIS_ADS_ERR_NOMAL', 1:
// 'CHASSIS_ADS_ERR_ADS_CAN_LOST', 2: 'CHASSIS_ADS_ERR_ADS_CAN_RECOVERY'},
// 'is_signed_var': False, 'len': 2, 'name': 'chassis_ads_err', 'offset': 0.0,
// 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'enum'}
Ecu_status_1_515::Chassis_ads_errType Ecustatus1515::chassis_ads_err(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 2);

  Ecu_status_1_515::Chassis_ads_errType ret =
      static_cast<Ecu_status_1_515::Chassis_ads_errType>(x);
  return ret;
}

// config detail: {'bit': 50, 'enum': {0: 'CHASSIS_BMS_CAN_NORMAL', 1:
// 'CHASSIS_BMS_CAN_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
// 'chassis_bms_can', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Ecu_status_1_515::Chassis_bms_canType Ecustatus1515::chassis_bms_can(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(2, 1);

  Ecu_status_1_515::Chassis_bms_canType ret =
      static_cast<Ecu_status_1_515::Chassis_bms_canType>(x);
  return ret;
}

// config detail: {'bit': 51, 'enum': {0: 'CHASSIS_EHB_CAN_NORMAL', 1:
// 'CHASSIS_EHB_CAN_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
// 'chassis_ehb_can', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Ecu_status_1_515::Chassis_ehb_canType Ecustatus1515::chassis_ehb_can(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(3, 1);

  Ecu_status_1_515::Chassis_ehb_canType ret =
      static_cast<Ecu_status_1_515::Chassis_ehb_canType>(x);
  return ret;
}

// config detail: {'bit': 52, 'enum': {0: 'CHASSIS_EHB_ERR_NORMAL', 1:
// 'CHASSIS_EHB_ERR_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
// 'chassis_ehb_err', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Ecu_status_1_515::Chassis_ehb_errType Ecustatus1515::chassis_ehb_err(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 1);

  Ecu_status_1_515::Chassis_ehb_errType ret =
      static_cast<Ecu_status_1_515::Chassis_ehb_errType>(x);
  return ret;
}

// config detail: {'bit': 53, 'enum': {0: 'CHASSIS_EPS_CAN_NORMAL', 1:
// 'CHASSIS_EPS_CAN_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
// 'chassis_eps_can', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Ecu_status_1_515::Chassis_eps_canType Ecustatus1515::chassis_eps_can(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  Ecu_status_1_515::Chassis_eps_canType ret =
      static_cast<Ecu_status_1_515::Chassis_eps_canType>(x);
  return ret;
}

// config detail: {'bit': 54, 'enum': {0: 'CHASSIS_EPS_ERR_NORMAL', 1:
// 'CHASSIS_EPS_ERR_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
// 'chassis_eps_err', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Ecu_status_1_515::Chassis_eps_errType Ecustatus1515::chassis_eps_err(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 1);

  Ecu_status_1_515::Chassis_eps_errType ret =
      static_cast<Ecu_status_1_515::Chassis_eps_errType>(x);
  return ret;
}

// config detail: {'bit': 55, 'enum': {0: 'CHASSIS_HW_LOST_NORMAL', 1:
// 'CHASSIS_HW_LOST_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
// 'chassis_hw_lost', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Ecu_status_1_515::Chassis_hw_lostType Ecustatus1515::chassis_hw_lost(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(7, 1);

  Ecu_status_1_515::Chassis_hw_lostType ret =
      static_cast<Ecu_status_1_515::Chassis_hw_lostType>(x);
  return ret;
}

// config detail: {'bit': 56, 'enum': {0: 'CHASSIS_MCU_CAN_NORMAL', 1:
// 'CHASSIS_MCU_CAN_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
// 'chassis_mcu_can', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Ecu_status_1_515::Chassis_mcu_canType Ecustatus1515::chassis_mcu_can(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 1);

  Ecu_status_1_515::Chassis_mcu_canType ret =
      static_cast<Ecu_status_1_515::Chassis_mcu_canType>(x);
  return ret;
}

// config detail: {'bit': 57, 'enum': {0: 'CHASSIS_MCU_ERR_NORMAL', 1:
// 'CHASSIS_MCU_ERR_ERROR'}, 'is_signed_var': False, 'len': 1, 'name':
// 'chassis_mcu_err', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Ecu_status_1_515::Chassis_mcu_errType Ecustatus1515::chassis_mcu_err(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(1, 1);

  Ecu_status_1_515::Chassis_mcu_errType ret =
      static_cast<Ecu_status_1_515::Chassis_mcu_errType>(x);
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
