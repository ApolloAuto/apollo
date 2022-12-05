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

#include "modules/canbus_vehicle/zhongyun/protocol/vehicle_state_feedback_c1.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

using ::apollo::drivers::canbus::Byte;

Vehiclestatefeedbackc1::Vehiclestatefeedbackc1() {}
const int32_t Vehiclestatefeedbackc1::ID = 0xC1;

void Vehiclestatefeedbackc1::Parse(const std::uint8_t* bytes, int32_t length,
                                   Zhongyun* chassis) const {
  chassis->mutable_vehicle_state_feedback_c1()->set_parking_actual(
      parking_actual(bytes, length));
  chassis->mutable_vehicle_state_feedback_c1()->set_brake_torque_feedback(
      brake_torque_feedback(bytes, length));
  chassis->mutable_vehicle_state_feedback_c1()->set_gear_state_actual(
      gear_state_actual(bytes, length));
  chassis->mutable_vehicle_state_feedback_c1()->set_steering_actual(
      steering_actual(bytes, length));
  chassis->mutable_vehicle_state_feedback_c1()->set_speed(
      speed(bytes, length) / 3.6);
}

// config detail: {'name': 'parking_actual', 'enum': {0:
// 'PARKING_ACTUAL_RELEASE', 1: 'PARKING_ACTUAL_PARKING_TRIGGER'},
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 56, 'type': 'enum', 'order': 'intel',
// 'physical_unit': ''}
Vehicle_state_feedback_c1::Parking_actualType
Vehiclestatefeedbackc1::parking_actual(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Vehicle_state_feedback_c1::Parking_actualType ret =
      static_cast<Vehicle_state_feedback_c1::Parking_actualType>(x);
  return ret;
}

// config detail: {'name': 'brake_torque_feedback', 'offset': 0.0, 'precision':
// 0.05, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit':
// 40, 'type': 'double', 'order': 'intel', 'physical_unit': '%'}
double Vehiclestatefeedbackc1::brake_torque_feedback(const std::uint8_t* bytes,
                                                     int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.050000;
  return ret;
}

// config detail: {'name': 'gear_state_actual', 'enum': {1:
// 'GEAR_STATE_ACTUAL_P', 2: 'GEAR_STATE_ACTUAL_N', 3: 'GEAR_STATE_ACTUAL_D', 4:
// 'GEAR_STATE_ACTUAL_R', 5: 'GEAR_STATE_ACTUAL_INVALID'}, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|5]',
// 'bit': 32, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vehicle_state_feedback_c1::Gear_state_actualType
Vehiclestatefeedbackc1::gear_state_actual(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Vehicle_state_feedback_c1::Gear_state_actualType ret =
      static_cast<Vehicle_state_feedback_c1::Gear_state_actualType>(x);
  return ret;
}

// config detail: {'name': 'steering_actual', 'offset': -1638.35, 'precision':
// 0.05, 'len': 16, 'is_signed_var': False, 'physical_range': '[-40|40]', 'bit':
// 16, 'type': 'double', 'order': 'intel', 'physical_unit': 'deg'}
double Vehiclestatefeedbackc1::steering_actual(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.050000 + -1638.350000;
  return ret;
}

// config detail: {'name': 'speed', 'offset': 0.0, 'precision': 0.01, 'len': 16,
// 'is_signed_var': False, 'physical_range': '[0|35]', 'bit': 0, 'type':
// 'double', 'order': 'intel', 'physical_unit': 'kph'}
double Vehiclestatefeedbackc1::speed(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}
}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
