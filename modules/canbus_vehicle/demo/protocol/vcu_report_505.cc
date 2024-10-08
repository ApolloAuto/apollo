/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/demo/protocol/vcu_report_505.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace demo {

using ::apollo::drivers::canbus::Byte;

Vcureport505::Vcureport505() {}
const int32_t Vcureport505::ID = 0x505;

void Vcureport505::Parse(const std::uint8_t* bytes, int32_t length,
                         Demo* chassis) const {
  chassis->mutable_vcu_report_505()->set_vehicle_mode_state(
      vehicle_mode_state(bytes, length));
  chassis->mutable_vcu_report_505()->set_aeb_mode(aeb_mode(bytes, length));
  chassis->mutable_vcu_report_505()->set_brake_light_actual(
      brake_light_actual(bytes, length));
  chassis->mutable_vcu_report_505()->set_turn_light_actual(
      turn_light_actual(bytes, length));
  chassis->mutable_vcu_report_505()->set_chassis_errcode(
      chassis_errcode(bytes, length));
  chassis->mutable_vcu_report_505()->set_drive_mode_sts(
      drive_mode_sts(bytes, length));
  chassis->mutable_vcu_report_505()->set_steer_mode_sts(
      steer_mode_sts(bytes, length));
  chassis->mutable_vcu_report_505()->set_frontcrash_state(
      frontcrash_state(bytes, length));
  chassis->mutable_vcu_report_505()->set_backcrash_state(
      backcrash_state(bytes, length));
  chassis->mutable_vcu_report_505()->set_aeb_brake_state(
      aeb_brake_state(bytes, length));
  chassis->mutable_vcu_report_505()->set_acc(acc(bytes, length));
  chassis->mutable_vcu_report_505()->set_speed(speed(bytes, length));
}

// config detail: {'bit': 36, 'enum': {0:
// 'VEHICLE_MODE_STATE_MANUAL_REMOTE_MODE', 1: 'VEHICLE_MODE_STATE_AUTO_MODE',
// 2: 'VEHICLE_MODE_STATE_EMERGENCY_MODE', 3:
// 'VEHICLE_MODE_STATE_STANDBY_MODE'}, 'is_signed_var': False, 'len': 2, 'name':
// 'vehicle_mode_state', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Vcu_report_505::Vehicle_mode_stateType Vcureport505::vehicle_mode_state(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 2);

  Vcu_report_505::Vehicle_mode_stateType ret =
      static_cast<Vcu_report_505::Vehicle_mode_stateType>(x);
  return ret;
}

// config detail: {'bit': 58, 'description': 'describle the vehicle AEB mode
// whether was set', 'enum': {0: 'AEB_MODE_DISABLE', 1: 'AEB_MODE_ENABLE'},
// 'is_signed_var': False, 'len': 1, 'name': 'aeb_mode', 'offset': 0.0, 'order':
// 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0,
// 'type': 'enum'}
Vcu_report_505::Aeb_modeType Vcureport505::aeb_mode(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(2, 1);

  Vcu_report_505::Aeb_modeType ret =
      static_cast<Vcu_report_505::Aeb_modeType>(x);
  return ret;
}

// config detail: {'bit': 11, 'enum': {0: 'BRAKE_LIGHT_ACTUAL_BRAKELIGHT_OFF',
// 1: 'BRAKE_LIGHT_ACTUAL_BRAKELIGHT_ON'}, 'is_signed_var': False, 'len': 1,
// 'name': 'brake_light_actual', 'offset': 0.0, 'order': 'motorola',
// 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Vcu_report_505::Brake_light_actualType Vcureport505::brake_light_actual(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  Vcu_report_505::Brake_light_actualType ret =
      static_cast<Vcu_report_505::Brake_light_actualType>(x);
  return ret;
}

// config detail: {'bit': 57, 'enum': {0: 'TURN_LIGHT_ACTUAL_TURNLAMPSTS_OFF',
// 1: 'TURN_LIGHT_ACTUAL_LEFT_TURNLAMPSTS_ON', 2:
// 'TURN_LIGHT_ACTUAL_RIGHT_TURNLAMPSTS_ON', 3:
// 'TURN_LIGHT_ACTUAL_HAZARD_WARNING_LAMPSTS_ON'}, 'is_signed_var': False,
// 'len': 2, 'name': 'turn_light_actual', 'offset': 0.0, 'order': 'motorola',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Vcu_report_505::Turn_light_actualType Vcureport505::turn_light_actual(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 2);

  Vcu_report_505::Turn_light_actualType ret =
      static_cast<Vcu_report_505::Turn_light_actualType>(x);
  return ret;
}

// config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name':
// 'chassis_errcode', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcureport505::chassis_errcode(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 39, 'enum': {0:
// 'DRIVE_MODE_STS_THROTTLE_PADDLE_DRIVE_MODE', 1:
// 'DRIVE_MODE_STS_SPEED_DRIVE_MODE'}, 'is_signed_var': False, 'len': 3, 'name':
// 'drive_mode_sts', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Vcu_report_505::Drive_mode_stsType Vcureport505::drive_mode_sts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(5, 3);

  Vcu_report_505::Drive_mode_stsType ret =
      static_cast<Vcu_report_505::Drive_mode_stsType>(x);
  return ret;
}

// config detail: {'bit': 10, 'enum': {0: 'STEER_MODE_STS_STANDARD_STEER_MODE',
// 1: 'STEER_MODE_STS_NON_DIRECTION_STEER_MODE', 2:
// 'STEER_MODE_STS_SYNC_DIRECTION_STEER_MODE'}, 'is_signed_var': False, 'len':
// 3, 'name': 'steer_mode_sts', 'offset': 0.0, 'order': 'motorola',
// 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Vcu_report_505::Steer_mode_stsType Vcureport505::steer_mode_sts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 3);

  Vcu_report_505::Steer_mode_stsType ret =
      static_cast<Vcu_report_505::Steer_mode_stsType>(x);
  return ret;
}

// config detail: {'bit': 33, 'enum': {0: 'FRONTCRASH_STATE_NO_EVENT', 1:
// 'FRONTCRASH_STATE_CRASH_EVENT'}, 'is_signed_var': False, 'len': 1, 'name':
// 'frontcrash_state', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Vcu_report_505::Frontcrash_stateType Vcureport505::frontcrash_state(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 1);

  Vcu_report_505::Frontcrash_stateType ret =
      static_cast<Vcu_report_505::Frontcrash_stateType>(x);
  return ret;
}

// config detail: {'bit': 34, 'enum': {0: 'BACKCRASH_STATE_NO_EVENT', 1:
// 'BACKCRASH_STATE_CRASH_EVENT'}, 'is_signed_var': False, 'len': 1, 'name':
// 'backcrash_state', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Vcu_report_505::Backcrash_stateType Vcureport505::backcrash_state(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 1);

  Vcu_report_505::Backcrash_stateType ret =
      static_cast<Vcu_report_505::Backcrash_stateType>(x);
  return ret;
}

// config detail: {'bit': 32, 'description': 'describe the vehicle e-brake
// command whether was triggered by AEB', 'enum': {0:
// 'AEB_BRAKE_STATE_INACTIVE', 1: 'AEB_BRAKE_STATE_ACTIVE'}, 'is_signed_var':
// False, 'len': 1, 'name': 'aeb_brake_state', 'offset': 0.0, 'order':
// 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
// 'signal_type': 'command', 'type': 'enum'}
Vcu_report_505::Aeb_brake_stateType Vcureport505::aeb_brake_state(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  Vcu_report_505::Aeb_brake_stateType ret =
      static_cast<Vcu_report_505::Aeb_brake_stateType>(x);
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': True, 'len': 12, 'name': 'acc',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-10|10]',
// 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
double Vcureport505::acc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 23, 'description': 'speed', 'is_signed_var': True,
// 'len': 16, 'name': 'speed', 'offset': 0.0, 'order': 'motorola',
// 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision':
// 0.001, 'signal_type': 'speed', 'type': 'double'}
double Vcureport505::speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}
}  // namespace demo
}  // namespace canbus
}  // namespace apollo
