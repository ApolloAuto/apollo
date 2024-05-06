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

#include "modules/canbus_vehicle/neolix_edu/protocol/vcu_brake_report_47.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

Vcubrakereport47::Vcubrakereport47() {}
const int32_t Vcubrakereport47::ID = 0x47;

void Vcubrakereport47::Parse(const std::uint8_t* bytes, int32_t length,
                             Neolix_edu* chassis) const {
  chassis->mutable_vcu_brake_report_47()->set_brake_enable_resp(
      brake_enable_resp(bytes, length));
  chassis->mutable_vcu_brake_report_47()->set_control_mode_resp(
      control_mode_resp(bytes, length));
  chassis->mutable_vcu_brake_report_47()->set_vcu_real_brake_valid(
      vcu_real_brake_valid(bytes, length));
  chassis->mutable_vcu_brake_report_47()->set_vcu_real_brake(
      vcu_real_brake(bytes, length));
  chassis->mutable_vcu_brake_report_47()->set_vcu_real_parking_status(
      vcu_real_parking_status(bytes, length));
  chassis->mutable_vcu_brake_report_47()->set_vcu_real_parking_valid(
      vcu_real_parking_valid(bytes, length));
  chassis->mutable_vcu_brake_report_47()->set_rampauxiliaryindication(
      rampauxiliaryindication(bytes, length));
  chassis->mutable_vcu_brake_report_47()->set_vehicleslope(
      vehicleslope(bytes, length));
  chassis->mutable_vcu_brake_report_47()->set_vcu_brakerept_alivecounter(
      vcu_brakerept_alivecounter(bytes, length));
  chassis->mutable_vcu_brake_report_47()->set_vcu_brakerept_checksum(
      vcu_brakerept_checksum(bytes, length));
  chassis->mutable_vcu_brake_report_47()->set_vcu_ehb_brake_state(
      vcu_ehb_brake_state(bytes, length));

  chassis->mutable_brake()->set_brake_pedal_position(
      vcu_real_brake(bytes, length));
  if (vcu_real_parking_status(bytes, length) == 1)
    chassis->mutable_epb()->set_parking_brake_status(Epb::PBRAKE_ON);
  if (vcu_real_parking_status(bytes, length) == 0)
    chassis->mutable_epb()->set_parking_brake_status(Epb::PBRAKE_OFF);
}

// config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'brake_enable_resp', 'is_signed_var':
// False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': ''}
bool Vcubrakereport47::brake_enable_resp(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Standby;0x1:auto drive;0x2:net
// drive;0x3:remote control;0x4:emergency brake;0x5~0x7:Reserved', 'enum': {0:
// 'CONTROL_MODE_RESP_STANDBY', 1: 'CONTROL_MODE_RESP_AUTO_DRIVE', 2:
// 'CONTROL_MODE_RESP_NET_DRIVE', 3: 'CONTROL_MODE_RESP_REMOTE_CONTROL', 4:
// 'CONTROL_MODE_RESP_EMERGENCY_BRAKE'}, 'precision': 1.0, 'len': 3, 'name':
// 'control_mode_resp', 'is_signed_var': False, 'offset': 0.0, 'physical_range':
// '[0|7]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit':
// 'bit'}
Vcu_brake_report_47::Control_mode_respType Vcubrakereport47::control_mode_resp(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 3);

  Vcu_brake_report_47::Control_mode_respType ret =
      static_cast<Vcu_brake_report_47::Control_mode_respType>(x);
  return ret;
}

// config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'vcu_real_brake_valid', 'is_signed_var':
// False, 'physical_range': '[0|0]', 'bit': 7, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': ''}
bool Vcubrakereport47::vcu_real_brake_valid(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'vcu_real_brake', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 15,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vcubrakereport47::vcu_real_brake(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description':
// '0x0:EPB_Released;0x1:EPB_Applied;0x2:EPB_Releasing;0x3:EPB_Fault;0x4:EPB_Applying',
// 'offset': 0.0, 'precision': 1.0, 'len': 3, 'name': 'vcu_real_parking_status',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 18, 'type': 'int',
// 'order': 'motorola', 'physical_unit': ''}
int Vcubrakereport47::vcu_real_parking_status(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 3);

  int ret = x;
  return ret;
}

// config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'vcu_real_parking_valid',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 19, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Vcubrakereport47::vcu_real_parking_valid(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:off;0x1:on', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'rampauxiliaryindication',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 20, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Vcubrakereport47::rampauxiliaryindication(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'vehicleslope', 'offset': 0.0, 'precision':
// 0.1758125, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|45]',
// 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit':
// '\xc2\xb0'}
double Vcubrakereport47::vehicleslope(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.175813;
  return ret;
}

// config detail: {'bit': 32, 'description': 'the vcu brake was caused reason',
// 'enum': {0: 'VCU_EHB_NORMAL_BRAKE', 1: 'VCU_EHB_BACKUP_REMOTE_BRAKE',
// 2: 'VCU_EHB_EMERGENCY_BUTTON_BRAKE', 3: 'VCU_EHB_ULTR_BRAKE', 4:
// 'VCU_EHB_BUMPER_BRAKE'}, 'is_signed_var': False, 'len': 3, 'name':
// 'vcu_ehb_brake_state', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|4]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Vcu_brake_report_47::Vcu_ehb_brakeType Vcubrakereport47::vcu_ehb_brake_state(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 3);

  Vcu_brake_report_47::Vcu_ehb_brakeType ret =
      static_cast<Vcu_brake_report_47::Vcu_ehb_brakeType>(x);
  return ret;
}

// config detail: {'name': 'vcu_brakerept_alivecounter', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vcubrakereport47::vcu_brakerept_alivecounter(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_brakerept_checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vcubrakereport47::vcu_brakerept_checksum(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
