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

#include "modules/canbus_vehicle/lincoln/protocol/gps_6f.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Gps6f::ID = 0x6F;

void Gps6f::Parse(const std::uint8_t *bytes, int32_t length,
                  Lincoln *chassis_detail) const {
  chassis_detail->mutable_basic()->set_altitude(altitude(bytes, length));
  chassis_detail->mutable_basic()->set_heading(heading(bytes, length));
  // speed mph -> mps
  chassis_detail->mutable_basic()->set_gps_speed(speed(bytes, length) *
                                                 0.44704);
  chassis_detail->mutable_basic()->set_hdop(hdop(bytes, length));
  chassis_detail->mutable_basic()->set_vdop(vdop(bytes, length));
  switch (fix_quality(bytes, length)) {
    case 0:
      chassis_detail->mutable_basic()->set_quality(FIX_NO);
      break;
    case 1:
      chassis_detail->mutable_basic()->set_quality(FIX_2D);
      break;
    case 2:
      chassis_detail->mutable_basic()->set_quality(FIX_3D);
      break;
    default:
      chassis_detail->mutable_basic()->set_quality(FIX_INVALID);
      break;
  }
  chassis_detail->mutable_basic()->set_num_satellites(
      num_satellites(bytes, length));
}

double Gps6f::altitude(const std::uint8_t *bytes, int32_t length) const {
  Byte high_frame(bytes + 1);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes + 0);
  int32_t low = low_frame.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  if (value > 0x7FFF) {
    value -= 0x10000;
  }
  return value * 0.250000;
}

double Gps6f::heading(const std::uint8_t *bytes, int32_t length) const {
  Byte high_frame(bytes + 3);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes + 2);
  int32_t low = low_frame.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value * 0.010000;
}

int32_t Gps6f::speed(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 4);
  int32_t x = frame.get_byte(0, 8);
  return x;
}

double Gps6f::hdop(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 5);
  int32_t x = frame.get_byte(0, 5);
  return x * 0.200000;
}

double Gps6f::vdop(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 6);
  int32_t x = frame.get_byte(0, 5);
  return x * 0.200000;
}

int32_t Gps6f::fix_quality(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 7);
  int32_t x = frame.get_byte(0, 3);
  return x;
}

int32_t Gps6f::num_satellites(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 7);
  int32_t x = frame.get_byte(3, 5);
  return x;
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
