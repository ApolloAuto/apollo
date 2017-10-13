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

#include "modules/canbus/vehicle/lincoln/protocol/tirepressure_71.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Tirepressure71::ID = 0x71;

void Tirepressure71::Parse(const std::uint8_t *bytes, int32_t length,
                           ChassisDetail *chassis_detail) const {
  chassis_detail->mutable_safety()->set_front_left_tire_press(
      front_left_tire(bytes, length));
  chassis_detail->mutable_safety()->set_front_right_tire_press(
      front_right_tire(bytes, length));
  chassis_detail->mutable_safety()->set_rear_left_tire_press(
      rear_left_tire(bytes, length));
  chassis_detail->mutable_safety()->set_rear_right_tire_press(
      rear_right_tire(bytes, length));
}

int32_t Tirepressure71::front_left_tire(const std::uint8_t *bytes,
                                        int32_t length) const {
  Byte high_frame(bytes + 1);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes + 0);
  int32_t low = low_frame.get_byte(0, 8);
  return (high << 8) | low;
}

int32_t Tirepressure71::front_right_tire(const std::uint8_t *bytes,
                                         int32_t length) const {
  Byte high_frame(bytes + 3);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes + 2);
  int32_t low = low_frame.get_byte(0, 8);
  return (high << 8) | low;
}

int32_t Tirepressure71::rear_left_tire(const std::uint8_t *bytes,
                                       int32_t length) const {
  Byte high_frame(bytes + 5);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes + 4);
  int32_t low = low_frame.get_byte(0, 8);
  return (high << 8) | low;
}

int32_t Tirepressure71::rear_right_tire(const std::uint8_t *bytes,
                                        int32_t length) const {
  Byte high_frame(bytes + 7);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes + 6);
  int32_t low = low_frame.get_byte(0, 8);
  return (high << 8) | low;
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
