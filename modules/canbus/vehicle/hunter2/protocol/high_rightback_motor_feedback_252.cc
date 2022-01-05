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

#include "modules/canbus/vehicle/hunter2/protocol/high_rightback_motor_feedback_252.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

using ::apollo::drivers::canbus::Byte;

Highrightbackmotorfeedback252::Highrightbackmotorfeedback252() {}
const int32_t Highrightbackmotorfeedback252::ID = 0x252;

void Highrightbackmotorfeedback252::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_hunter2()->mutable_high_rightback_motor_feedback_252()->set_high_motor_position(high_motor_position(bytes, length));
  chassis->mutable_hunter2()->mutable_high_rightback_motor_feedback_252()->set_high_motor_current(high_motor_current(bytes, length));
  chassis->mutable_hunter2()->mutable_high_rightback_motor_feedback_252()->set_high_motor_rpm(high_motor_rpm(bytes, length));
}

// config detail: {'bit': 39, 'is_signed_var': True, 'len': 32, 'name': 'high_motor_position', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': 'PN', 'precision': 1.0, 'type': 'int'}
int Highrightbackmotorfeedback252::high_motor_position(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 6);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 7);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 0;
  x >>= 0;

  int ret = x;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'high_motor_current', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-3276.8|3276.7]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double Highrightbackmotorfeedback252::high_motor_current(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': True, 'len': 16, 'name': 'high_motor_rpm', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32768|32767]', 'physical_unit': 'RPM', 'precision': 1.0, 'type': 'int'}
int Highrightbackmotorfeedback252::high_motor_rpm(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}
}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo
