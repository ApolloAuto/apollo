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

#include "modules/canbus/vehicle/hunter2/protocol/motion_control_feedback_221.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

using ::apollo::drivers::canbus::Byte;

Motioncontrolfeedback221::Motioncontrolfeedback221() {}
const int32_t Motioncontrolfeedback221::ID = 0x221;

void Motioncontrolfeedback221::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_hunter2()->mutable_motion_control_feedback_221()->set_steer(steer(bytes, length));
  chassis->mutable_hunter2()->mutable_motion_control_feedback_221()->set_speed(speed(bytes, length));
}

// config detail: {'bit': 55, 'is_signed_var': True, 'len': 16, 'name': 'steer', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'rad', 'precision': 0.001, 'type': 'double'}
double Motioncontrolfeedback221::steer(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': True, 'len': 16, 'name': 'speed', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
double Motioncontrolfeedback221::speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}
}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo
