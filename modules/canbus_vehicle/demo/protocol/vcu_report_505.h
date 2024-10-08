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

#pragma once

#include "modules/canbus_vehicle/demo/proto/demo.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace demo {

class Vcureport505
    : public ::apollo::drivers::canbus::ProtocolData<::apollo::canbus::Demo> {
 public:
  static const int32_t ID;
  Vcureport505();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Demo* chassis) const override;

 private:
  // config detail: {'bit': 36, 'enum': {0:
  // 'VEHICLE_MODE_STATE_MANUAL_REMOTE_MODE', 1: 'VEHICLE_MODE_STATE_AUTO_MODE',
  // 2: 'VEHICLE_MODE_STATE_EMERGENCY_MODE', 3:
  // 'VEHICLE_MODE_STATE_STANDBY_MODE'}, 'is_signed_var': False, 'len': 2,
  // 'name': 'Vehicle_Mode_State', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Vcu_report_505::Vehicle_mode_stateType vehicle_mode_state(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 58, 'description': 'describle the vehicle AEB mode
  // whether was set', 'enum': {0: 'AEB_MODE_DISABLE', 1: 'AEB_MODE_ENABLE'},
  // 'is_signed_var': False, 'len': 1, 'name': 'AEB_Mode', 'offset': 0.0,
  // 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  Vcu_report_505::Aeb_modeType aeb_mode(const std::uint8_t* bytes,
                                        const int32_t length) const;

  // config detail: {'bit': 11, 'enum': {0: 'BRAKE_LIGHT_ACTUAL_BRAKELIGHT_OFF',
  // 1: 'BRAKE_LIGHT_ACTUAL_BRAKELIGHT_ON'}, 'is_signed_var': False, 'len': 1,
  // 'name': 'Brake_Light_Actual', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Vcu_report_505::Brake_light_actualType brake_light_actual(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 57, 'enum': {0: 'TURN_LIGHT_ACTUAL_TURNLAMPSTS_OFF',
  // 1: 'TURN_LIGHT_ACTUAL_LEFT_TURNLAMPSTS_ON', 2:
  // 'TURN_LIGHT_ACTUAL_RIGHT_TURNLAMPSTS_ON', 3:
  // 'TURN_LIGHT_ACTUAL_HAZARD_WARNING_LAMPSTS_ON'}, 'is_signed_var': False,
  // 'len': 2, 'name': 'Turn_Light_Actual', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Vcu_report_505::Turn_light_actualType turn_light_actual(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name':
  // 'Chassis_errcode', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassis_errcode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 39, 'enum': {0:
  // 'DRIVE_MODE_STS_THROTTLE_PADDLE_DRIVE_MODE', 1:
  // 'DRIVE_MODE_STS_SPEED_DRIVE_MODE'}, 'is_signed_var': False, 'len': 3,
  // 'name': 'Drive_Mode_STS', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Vcu_report_505::Drive_mode_stsType drive_mode_sts(const std::uint8_t* bytes,
                                                    const int32_t length) const;

  // config detail: {'bit': 10, 'enum': {0:
  // 'STEER_MODE_STS_STANDARD_STEER_MODE', 1:
  // 'STEER_MODE_STS_NON_DIRECTION_STEER_MODE', 2:
  // 'STEER_MODE_STS_SYNC_DIRECTION_STEER_MODE'}, 'is_signed_var': False, 'len':
  // 3, 'name': 'Steer_Mode_STS', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Vcu_report_505::Steer_mode_stsType steer_mode_sts(const std::uint8_t* bytes,
                                                    const int32_t length) const;

  // config detail: {'bit': 33, 'enum': {0: 'FRONTCRASH_STATE_NO_EVENT', 1:
  // 'FRONTCRASH_STATE_CRASH_EVENT'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'FrontCrash_State', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Vcu_report_505::Frontcrash_stateType frontcrash_state(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 34, 'enum': {0: 'BACKCRASH_STATE_NO_EVENT', 1:
  // 'BACKCRASH_STATE_CRASH_EVENT'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'BackCrash_State', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Vcu_report_505::Backcrash_stateType backcrash_state(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'description': 'describe the vehicle e-brake
  // command whether was triggered by AEB', 'enum': {0:
  // 'AEB_BRAKE_STATE_INACTIVE', 1: 'AEB_BRAKE_STATE_ACTIVE'}, 'is_signed_var':
  // False, 'len': 1, 'name': 'AEB_Brake_State', 'offset': 0.0, 'order':
  // 'motorola', 'physical_range': '[0|0]', 'physical_unit': '',
  // 'precision': 1.0, 'signal_type': 'command', 'type': 'enum'}
  Vcu_report_505::Aeb_brake_stateType aeb_brake_state(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 7, 'is_signed_var': True, 'len': 12, 'name': 'ACC',
  // 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-10|10]',
  // 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
  double acc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 23, 'description': 'speed', 'is_signed_var': True,
  // 'len': 16, 'name': 'SPEED', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision':
  // 0.001, 'signal_type': 'speed', 'type': 'double'}
  double speed(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace demo
}  // namespace canbus
}  // namespace apollo
