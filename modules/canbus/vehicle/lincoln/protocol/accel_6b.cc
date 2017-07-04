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

#include "modules/canbus/vehicle/lincoln/protocol/accel_6b.h"

#include "modules/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

const int32_t Accel6b::ID = 0x6B;

void Accel6b::Parse(const std::uint8_t* bytes, int32_t length,
                    ChassisDetail* chassis_detail) const {
  chassis_detail->mutable_vehicle_spd()->set_lat_acc(
      lateral_acceleration(bytes, length));
  chassis_detail->mutable_vehicle_spd()->set_long_acc(
      longitudinal_acceleration(bytes, length));
  chassis_detail->mutable_vehicle_spd()->set_vert_acc(
      vertical_acceleration(bytes, length));
}

double Accel6b::lateral_acceleration(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte high_frame(bytes + 1);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes + 0);
  int32_t low = low_frame.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  if (value > 0x7FFF) {
    value -= 0x10000;
  }
  return value * 0.010000;
}

double Accel6b::longitudinal_acceleration(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte high_frame(bytes + 3);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes + 2);
  int32_t low = low_frame.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  if (value > 0x7FFF) {
    value -= 0x10000;
  }
  return value * 0.010000;
}

double Accel6b::vertical_acceleration(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte high_frame(bytes + 5);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes + 4);
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
