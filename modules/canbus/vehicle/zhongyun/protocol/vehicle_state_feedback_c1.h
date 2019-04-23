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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

class Vehiclestatefeedbackc1 : public ::apollo::drivers::canbus::ProtocolData<
                                   ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vehiclestatefeedbackc1();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'Parking_actual', 'enum': {0:
  // 'PARKING_ACTUAL_RELEASE', 1: 'PARKING_ACTUAL_PARKING_TRIGGER'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 56, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  Vehicle_state_feedback_c1::Parking_actualType parking_actual(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'brake_torque_feedback', 'offset': 0.0,
  // 'precision': 0.05, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[0|100]', 'bit': 40, 'type': 'double', 'order': 'intel', 'physical_unit':
  // '%'}
  double brake_torque_feedback(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'name': 'Gear_state_actual', 'enum': {1:
  // 'GEAR_STATE_ACTUAL_P', 2: 'GEAR_STATE_ACTUAL_N', 3: 'GEAR_STATE_ACTUAL_D',
  // 4: 'GEAR_STATE_ACTUAL_R', 5: 'GEAR_STATE_ACTUAL_INVALID'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|5]', 'bit': 32, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  Vehicle_state_feedback_c1::Gear_state_actualType gear_state_actual(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Steering_actual', 'offset': -1638.35, 'precision':
  // 0.05, 'len': 16, 'is_signed_var': False, 'physical_range': '[-40|40]',
  // 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': 'deg'}
  double steering_actual(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'speed', 'offset': 0.0, 'precision': 0.01, 'len':
  // 16, 'is_signed_var': False, 'physical_range': '[0|35]', 'bit': 0, 'type':
  // 'double', 'order': 'intel', 'physical_unit': 'kph'}
  double speed(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
