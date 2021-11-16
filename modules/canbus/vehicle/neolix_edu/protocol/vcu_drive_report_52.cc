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

#include "modules/canbus/vehicle/neolix_edu/protocol/vcu_drive_report_52.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

Vcudrivereport52::Vcudrivereport52() {}
const int32_t Vcudrivereport52::ID = 0x52;

void Vcudrivereport52::Parse(const std::uint8_t* bytes, int32_t length,
                             ChassisDetail* chassis) const {
  chassis->mutable_neolix_edu()
      ->mutable_vcu_drive_report_52()
      ->set_drive_enable_resp(drive_enable_resp(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_drive_report_52()
      ->set_control_mode_resp(control_mode_resp(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_drive_report_52()
      ->set_vcu_real_shift(vcu_real_shift(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_drive_report_52()
      ->set_vcu_real_shift_valid(vcu_real_shift_valid(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_drive_report_52()
      ->set_vcu_real_torque_valid(vcu_real_torque_valid(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_drive_report_52()
      ->set_vcu_real_torque(vcu_real_torque(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_drive_report_52()
      ->set_vcu_limitedtorquemode(vcu_limitedtorquemode(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_drive_report_52()
      ->set_vcu_driverept_alivecounter(
          vcu_driverept_alivecounter(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_drive_report_52()
      ->set_vcu_driverept_checksum(vcu_driverept_checksum(bytes, length));

  chassis->mutable_safety()->set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  chassis->mutable_gas()->set_gas_pedal_position(
      vcu_real_torque(bytes, length));
  chassis->mutable_gear()->set_gear_state(
      (apollo::canbus::Chassis_GearPosition)vcu_real_shift(bytes, length));
  chassis->mutable_check_response()->set_is_vcu_online(
      drive_enable_resp(bytes, length) == 1);
}

// config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'drive_enable_resp', 'is_signed_var':
// False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': ''}
bool Vcudrivereport52::drive_enable_resp(const std::uint8_t* bytes,
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
// '[0|7]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Vcu_drive_report_52::Control_mode_respType Vcudrivereport52::control_mode_resp(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 3);

  Vcu_drive_report_52::Control_mode_respType ret =
      static_cast<Vcu_drive_report_52::Control_mode_respType>(x);
  return ret;
}

// config detail: {'description':
// '0x0:N\xe6\xa1\xa3;0x1:D\xe6\xa1\xa3;0x2:R\xe6\xa1\xa3;0x3:Reserved', 'enum':
// {0: 'VCU_REAL_SHIFT_N', 1: 'VCU_REAL_SHIFT_D', 2: 'VCU_REAL_SHIFT_R', 3:
// 'VCU_REAL_SHIFT_RESERVED'}, 'precision': 1.0, 'len': 2, 'name':
// 'vcu_real_shift', 'is_signed_var': False, 'offset': 0.0, 'physical_range':
// '[0|3]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Vcu_drive_report_52::Vcu_real_shiftType Vcudrivereport52::vcu_real_shift(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 2);

  Vcu_drive_report_52::Vcu_real_shiftType ret =
      static_cast<Vcu_drive_report_52::Vcu_real_shiftType>(x);
  return ret;
}

// config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'vcu_real_shift_valid', 'is_signed_var':
// False, 'physical_range': '[0|0]', 'bit': 10, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': ''}
bool Vcudrivereport52::vcu_real_shift_valid(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'vcu_real_torque_valid', 'is_signed_var':
// False, 'physical_range': '[0|0]', 'bit': 11, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': ''}
bool Vcudrivereport52::vcu_real_torque_valid(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'vcu_real_torque', 'offset': -665.0, 'precision':
// 0.02, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Nm'}
double Vcudrivereport52::vcu_real_torque(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.020000 + -665.000000;
  return ret;
}

// config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'vcu_limitedtorquemode', 'is_signed_var':
// False, 'physical_range': '[0|0]', 'bit': 32, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': ''}
bool Vcudrivereport52::vcu_limitedtorquemode(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'vcu_driverept_alivecounter', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vcudrivereport52::vcu_driverept_alivecounter(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_driverept_checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vcudrivereport52::vcu_driverept_checksum(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
