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

#include "modules/canbus/vehicle/lincoln/protocol/gyro_6c.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Gyro6c::ID = 0x6C;

void Gyro6c::Parse(const std::uint8_t *bytes, int32_t length,
                   ChassisDetail *chassis_detail) const {
  chassis_detail->mutable_vehicle_spd()->set_roll_rate(
      roll_rate(bytes, length));
  chassis_detail->mutable_vehicle_spd()->set_yaw_rate(yaw_rate(bytes, length));
  // why
  chassis_detail->mutable_vehicle_spd()->set_is_yaw_rate_valid(true);
}

double Gyro6c::roll_rate(const std::uint8_t *bytes, int32_t length) const {
  Byte high_frame(bytes + 1);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes + 0);
  int32_t low = low_frame.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  if (value > 0x7FFF) {
    value -= 0x10000;
  }
  return value * 0.000200;
}

double Gyro6c::yaw_rate(const std::uint8_t *bytes, int32_t length) const {
  Byte high_frame(bytes + 3);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes + 2);
  int32_t low = low_frame.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  if (value > 0x7FFF) {
    value -= 0x10000;
  }
  return value * 0.000200;
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
