/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/lincoln/protocol/accel_6b.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Accel6b::ID = 0x6B;

void Accel6b::Parse(const std::uint8_t *bytes, int32_t length,
                    Lincoln *chassis_detail) const {
  chassis_detail->mutable_vehicle_spd()->set_lat_acc(
      lateral_acceleration(bytes, length));
  chassis_detail->mutable_vehicle_spd()->set_long_acc(
      longitudinal_acceleration(bytes, length));
  chassis_detail->mutable_vehicle_spd()->set_vert_acc(
      vertical_acceleration(bytes, length));
}

double Accel6b::lateral_acceleration(const std::uint8_t *bytes,
                                     const int32_t length) const {
  DCHECK_GE(length, 2);
  return parse_two_frames(bytes[0], bytes[1]);
}

double Accel6b::longitudinal_acceleration(const std::uint8_t *bytes,
                                          const int32_t length) const {
  DCHECK_GE(length, 4);
  return parse_two_frames(bytes[2], bytes[3]);
}

double Accel6b::vertical_acceleration(const std::uint8_t *bytes,
                                      const int32_t length) const {
  DCHECK_GE(length, 6);
  return parse_two_frames(bytes[4], bytes[5]);
}

double Accel6b::parse_two_frames(const std::uint8_t low_byte,
                                 const std::uint8_t high_byte) const {
  Byte high_frame(&high_byte);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(&low_byte);
  int32_t low = low_frame.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  if (value > 0x7FFF) {
    value -= 0x10000;
  }
  return value * 0.010000;
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
