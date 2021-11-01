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

#include "modules/canbus/vehicle/hunter2/protocol/low_steering_motor_feedback_261.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

using ::apollo::drivers::canbus::Byte;

Lowsteeringmotorfeedback261::Lowsteeringmotorfeedback261() {}
const int32_t Lowsteeringmotorfeedback261::ID = 0x261;

void Lowsteeringmotorfeedback261::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_hunter2()->mutable_low_steering_motor_feedback_261()->set_low_motordriver_status(low_motordriver_status(bytes, length));
  chassis->mutable_hunter2()->mutable_low_steering_motor_feedback_261()->set_low_motor_temperature(low_motor_temperature(bytes, length));
  chassis->mutable_hunter2()->mutable_low_steering_motor_feedback_261()->set_low_motordriver_temperature(low_motordriver_temperature(bytes, length));
  chassis->mutable_hunter2()->mutable_low_steering_motor_feedback_261()->set_low_motordirver_voltage(low_motordirver_voltage(bytes, length));
}

// config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name': 'low_motordriver_status', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Lowsteeringmotorfeedback261::low_motordriver_status(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': True, 'len': 8, 'name': 'low_motor_temperature', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-128|127]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Lowsteeringmotorfeedback261::low_motor_temperature(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'low_motordriver_temperature', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Lowsteeringmotorfeedback261::low_motordriver_temperature(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 16, 'name': 'low_motordirver_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|6553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
double Lowsteeringmotorfeedback261::low_motordirver_voltage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}
}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo
