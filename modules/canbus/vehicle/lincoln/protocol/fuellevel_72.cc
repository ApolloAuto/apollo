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

#include "modules/canbus/vehicle/lincoln/protocol/fuellevel_72.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Fuellevel72::ID = 0x72;

void Fuellevel72::Parse(const std::uint8_t *bytes, int32_t length,
                        ChassisDetail *chassis_detail) const {
  chassis_detail->mutable_battery()->set_fuel_level(fuel_level(bytes, length));
}

double Fuellevel72::fuel_level(const std::uint8_t *bytes,
                               int32_t length) const {
  Byte high_frame(bytes + 1);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes);
  int32_t low = low_frame.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  if (value > 0x7FFF) {
    value -= 0x10000;
  }
  // should be in range of
  // [0x0000, 0x0398]
  // or [0xfc68, 0xffff]
  const double fuel_level_coeff = 0.108696;
  return value * fuel_level_coeff;
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
