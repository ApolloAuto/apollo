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

#include "modules/canbus_vehicle/zhongyun/proto/zhongyun.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

class Enablestatefeedbackc3 : public ::apollo::drivers::canbus::ProtocolData<
                                  ::apollo::canbus::Zhongyun> {
 public:
  static const int32_t ID;
  Enablestatefeedbackc3();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Zhongyun* chassis) const override;

 private:
  // config detail: {'name': 'Parking_enable_state', 'enum': {0:
  // 'PARKING_ENABLE_STATE_PARKING_MANUALCONTROL', 1:
  // 'PARKING_ENABLE_STATE_PARKING_AUTOCONTROL', 2:
  // 'PARKING_ENABLE_STATE_PARKING_TAKEOVER'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit':
  // 32, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Enable_state_feedback_c3::Parking_enable_stateType parking_enable_state(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Steering_enable_state', 'enum': {0:
  // 'STEERING_ENABLE_STATE_STEERING_MANUALCONTROL', 1:
  // 'STEERING_ENABLE_STATE_STEERING_AUTOCONTROL', 2:
  // 'STEERING_ENABLE_STATE_STEERING_MANUAL_TAKEOVER'}, 'precision': 1.0, 'len':
  // 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit':
  // 8, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Enable_state_feedback_c3::Steering_enable_stateType steering_enable_state(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Gear_enable_actual', 'enum': {0:
  // 'GEAR_ENABLE_ACTUAL_GEAR_MANUALCONTROL', 1:
  // 'GEAR_ENABLE_ACTUAL_GEAR_AUTOCONTROL', 2:
  // 'GEAR_ENABLE_ACTUAL_GEAR_MANUAL_TAKEOVER'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 0,
  // 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Enable_state_feedback_c3::Gear_enable_actualType gear_enable_actual(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Driven_enable_state', 'enum': {0:
  // 'DRIVEN_ENABLE_STATE_DRIVE_MANUAL', 1: 'DRIVEN_ENABLE_STATE_DRIVE_AUTO', 2:
  // 'DRIVEN_ENABLE_STATE_DRIVE_TAKEOVER'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit':
  // 16, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Enable_state_feedback_c3::Driven_enable_stateType driven_enable_state(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Brake_enable_state', 'enum': {0:
  // 'BRAKE_ENABLE_STATE_BRAKE_MANUAL', 1: 'BRAKE_ENABLE_STATE_BRAKE_AUTO', 2:
  // 'BRAKE_ENABLE_STATE_BRAKE_TAKEOVER'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit':
  // 24, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Enable_state_feedback_c3::Brake_enable_stateType brake_enable_state(
      const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
