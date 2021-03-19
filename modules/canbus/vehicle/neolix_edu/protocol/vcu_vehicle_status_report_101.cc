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

#include "modules/canbus/vehicle/neolix_edu/protocol/vcu_vehicle_status_report_101.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

Vcuvehiclestatusreport101::Vcuvehiclestatusreport101() {}
const int32_t Vcuvehiclestatusreport101::ID = 0x101;

void Vcuvehiclestatusreport101::Parse(const std::uint8_t* bytes, int32_t length,
                                      ChassisDetail* chassis) const {
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_status_report_101()
      ->set_drive_enable_resp(drive_enable_resp(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_status_report_101()
      ->set_vcu_highvoltagecircuitstate(
          vcu_highvoltagecircuitstate(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_status_report_101()
      ->set_vcu_dcdc_enabledstates(vcu_dcdc_enabledstates(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_status_report_101()
      ->set_control_mode_resp(control_mode_resp(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_status_report_101()
      ->set_vcu_vehicle_speed(vcu_vehicle_speed(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_status_report_101()
      ->set_vcu_lowbatterychargingfunctionst(
          vcu_lowbatterychargingfunctionst(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_status_report_101()
      ->set_vcu_display_soc(vcu_display_soc(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_status_report_101()
      ->set_vcu_motor_speed(vcu_motor_speed(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_status_report_101()
      ->set_vcu_motor_direction(vcu_motor_direction(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_status_report_101()
      ->set_vcu_motor_speed_valid(vcu_motor_speed_valid(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_status_report_101()
      ->set_vcu_statusrept_alivecounter(
          vcu_statusrept_alivecounter(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_status_report_101()
      ->set_vcu_statusrept_checksum(vcu_statusrept_checksum(bytes, length));

  chassis->mutable_vehicle_spd()->set_vehicle_spd(
      vcu_vehicle_speed(bytes, length));
  chassis->mutable_battery()->set_battery_percent(
      vcu_display_soc(bytes, length));
}

// config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'drive_enable_resp', 'is_signed_var':
// False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': ''}
bool Vcuvehiclestatusreport101::drive_enable_resp(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Disconnect;0x1:Connect', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'vcu_highvoltagecircuitstate',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 2, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Vcuvehiclestatusreport101::vcu_highvoltagecircuitstate(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0: Disable;0x1:Enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'vcu_dcdc_enabledstates',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 3, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Vcuvehiclestatusreport101::vcu_dcdc_enabledstates(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

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
Vcu_vehicle_status_report_101::Control_mode_respType
Vcuvehiclestatusreport101::control_mode_resp(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 3);

  Vcu_vehicle_status_report_101::Control_mode_respType ret =
      static_cast<Vcu_vehicle_status_report_101::Control_mode_respType>(x);
  return ret;
}

// config detail: {'name': 'vcu_vehicle_speed', 'offset': 0.0, 'precision':
// 0.05625, 'len': 13, 'is_signed_var': False, 'physical_range': '[0|460.69]',
// 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Km/h'}
double Vcuvehiclestatusreport101::vcu_vehicle_speed(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'description': '0x0:Reserved;0x1:Start;0x2:Stop;0x3:Invalid
// ', 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name':
// 'vcu_lowbatterychargingfunctionst', 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 17, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vcuvehiclestatusreport101::vcu_lowbatterychargingfunctionst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_display_soc', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 31,
// 'type': 'int', 'order': 'motorola', 'physical_unit': '%'}
int Vcuvehiclestatusreport101::vcu_display_soc(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_motor_speed', 'offset': 0.0, 'precision': 0.25,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39,
// 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Vcuvehiclestatusreport101::vcu_motor_speed(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.250000;
  return ret;
}

// config detail: {'description': '0x0:Standby Status;0x1:Forward
// Mode;0x2:Reverse Mode', 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name':
// 'vcu_motor_direction', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 54, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vcuvehiclestatusreport101::vcu_motor_direction(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 2);

  int ret = x;
  return ret;
}

// config detail: {'description': '0x0:disable;0x1:enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'vcu_motor_speed_valid', 'is_signed_var':
// False, 'physical_range': '[0|0]', 'bit': 55, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': ''}
bool Vcuvehiclestatusreport101::vcu_motor_speed_valid(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'vcu_statusrept_alivecounter', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vcuvehiclestatusreport101::vcu_statusrept_alivecounter(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_statusrept_checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vcuvehiclestatusreport101::vcu_statusrept_checksum(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
