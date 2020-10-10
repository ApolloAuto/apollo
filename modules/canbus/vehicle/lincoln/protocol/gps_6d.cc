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

#include "modules/canbus/vehicle/lincoln/protocol/gps_6d.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Gps6d::ID = 0x6D;

void Gps6d::Parse(const std::uint8_t *bytes, int32_t length,
                  ChassisDetail *chassis_detail) const {
  chassis_detail->mutable_basic()->set_latitude(latitude(bytes, length));
  chassis_detail->mutable_basic()->set_longitude(longitude(bytes, length));
  chassis_detail->mutable_basic()->set_gps_valid(is_valid(bytes, length));
}

double Gps6d::latitude(const std::uint8_t *bytes, int32_t length) const {
  Byte frame_0(bytes + 3);
  int32_t value = frame_0.get_byte(0, 7);

  Byte frame_1(bytes + 2);
  int32_t t = frame_1.get_byte(0, 8);
  value <<= 8;
  value |= t;

  Byte frame_2(bytes + 1);
  t = frame_2.get_byte(0, 8);
  value <<= 8;
  value |= t;

  Byte frame_3(bytes);
  t = frame_3.get_byte(0, 8);
  value <<= 8;
  value |= t;

  if (value > 0x3FFFFFFF) {
    value -= 0x80000000;
  }

  return value * (1.000000 / 3.000000) * 1e-6;
}

double Gps6d::longitude(const std::uint8_t *bytes, int32_t length) const {
  Byte frame_0(bytes + 7);
  int32_t value = frame_0.get_byte(0, 7);

  Byte frame_1(bytes + 6);
  int32_t t = frame_1.get_byte(0, 8);
  value <<= 8;
  value |= t;

  Byte frame_2(bytes + 5);
  t = frame_2.get_byte(0, 8);
  value <<= 8;
  value |= t;

  Byte frame_3(bytes + 4);
  t = frame_3.get_byte(0, 8);
  value <<= 8;
  value |= t;

  if (value > 0x3FFFFFFF) {
    value -= 0x80000000;
  }

  return value * (1.000000 / 3.000000) * 1e-6;
}

bool Gps6d::is_valid(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(7);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
