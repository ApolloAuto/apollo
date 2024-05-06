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

#include "modules/canbus_vehicle/zhongyun/protocol/enable_state_feedback_c3.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

using ::apollo::drivers::canbus::Byte;

Enablestatefeedbackc3::Enablestatefeedbackc3() {}
const int32_t Enablestatefeedbackc3::ID = 0xC3;

void Enablestatefeedbackc3::Parse(const std::uint8_t* bytes, int32_t length,
                                  Zhongyun* chassis) const {
  chassis->mutable_enable_state_feedback_c3()->set_parking_enable_state(
      parking_enable_state(bytes, length));
  chassis->mutable_enable_state_feedback_c3()->set_steering_enable_state(
      steering_enable_state(bytes, length));
  chassis->mutable_enable_state_feedback_c3()->set_gear_enable_actual(
      gear_enable_actual(bytes, length));
  chassis->mutable_enable_state_feedback_c3()->set_driven_enable_state(
      driven_enable_state(bytes, length));
  chassis->mutable_enable_state_feedback_c3()->set_brake_enable_state(
      brake_enable_state(bytes, length));
}

// config detail: {'name': 'parking_enable_state', 'enum': {0:
// 'PARKING_ENABLE_STATE_PARKING_MANUALCONTROL', 1:
// 'PARKING_ENABLE_STATE_PARKING_AUTOCONTROL', 2:
// 'PARKING_ENABLE_STATE_PARKING_TAKEOVER'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32,
// 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Enable_state_feedback_c3::Parking_enable_stateType
Enablestatefeedbackc3::parking_enable_state(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Enable_state_feedback_c3::Parking_enable_stateType ret =
      static_cast<Enable_state_feedback_c3::Parking_enable_stateType>(x);
  return ret;
}

// config detail: {'name': 'steering_enable_state', 'enum': {0:
// 'STEERING_ENABLE_STATE_STEERING_MANUALCONTROL', 1:
// 'STEERING_ENABLE_STATE_STEERING_AUTOCONTROL', 2:
// 'STEERING_ENABLE_STATE_STEERING_MANUAL_TAKEOVER'}, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]',
// 'bit': 8, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Enable_state_feedback_c3::Steering_enable_stateType
Enablestatefeedbackc3::steering_enable_state(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Enable_state_feedback_c3::Steering_enable_stateType ret =
      static_cast<Enable_state_feedback_c3::Steering_enable_stateType>(x);
  return ret;
}

// config detail: {'name': 'gear_enable_actual', 'enum': {0:
// 'GEAR_ENABLE_ACTUAL_GEAR_MANUALCONTROL', 1:
// 'GEAR_ENABLE_ACTUAL_GEAR_AUTOCONTROL', 2:
// 'GEAR_ENABLE_ACTUAL_GEAR_MANUAL_TAKEOVER'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 0,
// 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Enable_state_feedback_c3::Gear_enable_actualType
Enablestatefeedbackc3::gear_enable_actual(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Enable_state_feedback_c3::Gear_enable_actualType ret =
      static_cast<Enable_state_feedback_c3::Gear_enable_actualType>(x);
  return ret;
}

// config detail: {'name': 'driven_enable_state', 'enum': {0:
// 'DRIVEN_ENABLE_STATE_DRIVE_MANUAL', 1: 'DRIVEN_ENABLE_STATE_DRIVE_AUTO',
// 2: 'DRIVEN_ENABLE_STATE_DRIVE_TAKEOVER'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 16,
// 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Enable_state_feedback_c3::Driven_enable_stateType
Enablestatefeedbackc3::driven_enable_state(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Enable_state_feedback_c3::Driven_enable_stateType ret =
      static_cast<Enable_state_feedback_c3::Driven_enable_stateType>(x);
  return ret;
}

// config detail: {'name': 'brake_enable_state', 'enum': {0:
// 'BRAKE_ENABLE_STATE_BRAKE_MANUAL', 1: 'BRAKE_ENABLE_STATE_BRAKE_AUTO', 2:
// 'BRAKE_ENABLE_STATE_BRAKE_TAKEOVER'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 24,
// 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Enable_state_feedback_c3::Brake_enable_stateType
Enablestatefeedbackc3::brake_enable_state(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Enable_state_feedback_c3::Brake_enable_stateType ret =
      static_cast<Enable_state_feedback_c3::Brake_enable_stateType>(x);
  return ret;
}
}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
