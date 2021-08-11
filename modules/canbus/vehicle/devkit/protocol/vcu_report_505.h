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
  // config detail: {'bit': 36, 'enum': {0:
  // 'VEHICLE_MODE_STATE_MANUAL_REMOTE_MODE', 1: 'VEHICLE_MODE_STATE_AUTO_MODE',
  // 2: 'VEHICLE_MODE_STATE_EMERGENCY_MODE', 3:
  // 'VEHICLE_MODE_STATE_STANDBY_MODE'}, 'is_signed_var': False, 'len': 2,
  // 'name': 'Vehicle_Mode_State', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Vcu_report_505::Vehicle_mode_stateType vehicle_mode_state(
      const std::uint8_t* bytes, const int32_t length) const;

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

  // config detail: {'bit': 32, 'enum': {0: 'AEB_STATE_INACTIVE', 1:
  // 'AEB_STATE_ACTIVE'}, 'is_signed_var': False, 'len': 1, 'name': 'AEB_State',
  // 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Vcu_report_505::Aeb_stateType aeb_state(const std::uint8_t* bytes,
                                          const int32_t length) const;

  // config detail: {'bit': 7, 'is_signed_var': True, 'len': 12, 'name': 'ACC',
  // 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-10|10]',
  // 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
  double acc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 23, 'is_signed_var': False, 'len': 16, 'name':
  // 'SPEED', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|65.535]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double speed(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
