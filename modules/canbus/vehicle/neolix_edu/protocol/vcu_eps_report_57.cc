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

#include "modules/canbus/vehicle/neolix_edu/protocol/vcu_eps_report_57.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

Vcuepsreport57::Vcuepsreport57() {}
const int32_t Vcuepsreport57::ID = 0x57;

void Vcuepsreport57::Parse(const std::uint8_t* bytes, int32_t length,
                           ChassisDetail* chassis) const {
  chassis->mutable_neolix_edu()
      ->mutable_vcu_eps_report_57()
      ->set_drive_enable_resp(drive_enable_resp(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_eps_report_57()
      ->set_control_mode_resp(control_mode_resp(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_eps_report_57()
      ->set_vcu_eps_report(vcu_eps_report(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_eps_report_57()
      ->set_vcu_real_angle(vcu_real_angle(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_eps_report_57()
      ->set_vcu_real_angle_valid(vcu_real_angle_valid(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_eps_report_57()
      ->set_vcu_target_angle_valid(vcu_target_angle_valid(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_eps_report_57()
      ->set_vcu_target_angle(vcu_target_angle(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_eps_report_57()
      ->set_vcu_eps_rept_alivecounter(vcu_eps_rept_alivecounter(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_eps_report_57()
      ->set_vcu_eps_rept_checksum(vcu_eps_rept_checksum(bytes, length));

  chassis->mutable_eps()->set_steering_angle(vcu_real_angle(bytes, length));
  chassis->mutable_check_response()->set_is_eps_online(
      drive_enable_resp(bytes, length) == 1);
}

// config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'drive_enable_resp', 'is_signed_var':
// False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': ''}
bool Vcuepsreport57::drive_enable_resp(const std::uint8_t* bytes,
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
Vcu_eps_report_57::Control_mode_respType Vcuepsreport57::control_mode_resp(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 3);

  Vcu_eps_report_57::Control_mode_respType ret =
      static_cast<Vcu_eps_report_57::Control_mode_respType>(x);
  return ret;
}

// config detail: {'description': '0x0:not overflow;0x1:overflow', 'offset':
// 0.0, 'precision': 1.0, 'len': 1, 'name': 'vcu_eps_report', 'is_signed_var':
// False, 'physical_range': '[0|0]', 'bit': 7, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': ''}
bool Vcuepsreport57::vcu_eps_report(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': ';', 'offset': -2048.0, 'precision': 0.0625,
// 'len': 16, 'name': 'vcu_real_angle', 'is_signed_var': False,
// 'physical_range': '[0|0]', 'bit': 23, 'type': 'double', 'order': 'motorola',
// 'physical_unit': ''}
double Vcuepsreport57::vcu_real_angle(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.062500 + -2048.000000;
  return -ret;
}

// config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'vcu_real_angle_valid', 'is_signed_var':
// False, 'physical_range': '[0|0]', 'bit': 32, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': ''}
bool Vcuepsreport57::vcu_real_angle_valid(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:disable;0x1:enable;', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'vcu_target_angle_valid',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 33, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Vcuepsreport57::vcu_target_angle_valid(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'vcu_target_angle', 'offset': -512.0, 'precision':
// 0.25, 'len': 12, 'is_signed_var': False, 'physical_range': '[-380|380]',
// 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Vcuepsreport57::vcu_target_angle(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.250000 + -512.000000;
  return ret;
}

// config detail: {'name': 'vcu_eps_rept_alivecounter', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vcuepsreport57::vcu_eps_rept_alivecounter(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_eps_rept_checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vcuepsreport57::vcu_eps_rept_checksum(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
