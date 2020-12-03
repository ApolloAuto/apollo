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
namespace devkit {

class Vcureport505 : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcureport505();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'Battery_Soc', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 47,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': '%'}
  int battery_soc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Vehicle_Mode_State', 'enum': {0:
  // 'VEHICLE_MODE_STATE_MANUAL_REMOTE_MODE', 1: 'VEHICLE_MODE_STATE_AUTO_MODE',
  // 2: 'VEHICLE_MODE_STATE_EMERGENCY_MODE', 3:
  // 'VEHICLE_MODE_STATE_STANDBY_MODE'}, 'precision': 1.0, 'len': 2,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 36, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vcu_report_505::Vehicle_mode_stateType vehicle_mode_state(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'FrontCrash_State', 'enum': {0:
  // 'FRONTCRASH_STATE_NO_EVENT', 1: 'FRONTCRASH_STATE_CRASH_EVENT'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 33, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Vcu_report_505::Frontcrash_stateType frontcrash_state(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BackCrash_State', 'enum': {0:
  // 'BACKCRASH_STATE_NO_EVENT', 1: 'BACKCRASH_STATE_CRASH_EVENT'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 34, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Vcu_report_505::Backcrash_stateType backcrash_state(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'AEB_State', 'enum': {0: 'AEB_STATE_INACTIVE', 1:
  // 'AEB_STATE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 32, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Vcu_report_505::Aeb_stateType aeb_state(const std::uint8_t* bytes,
                                          const int32_t length) const;

  // config detail: {'name': 'ACC', 'offset': 0.0, 'precision': 0.01, 'len': 12,
  // 'is_signed_var': True, 'physical_range': '[-10|10]', 'bit': 7, 'type':
  // 'double', 'order': 'motorola', 'physical_unit': 'm/s^2'}
  double acc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SPEED', 'offset': 0.0, 'precision': 0.001, 'len':
  // 16, 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit': 23,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
  double speed(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
