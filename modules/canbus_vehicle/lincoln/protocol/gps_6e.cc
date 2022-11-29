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

#include "modules/canbus_vehicle/lincoln/protocol/gps_6e.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Gps6e::ID = 0x6E;

void Gps6e::Parse(const std::uint8_t *bytes, int32_t length,
                  Lincoln *chassis_detail) const {
  chassis_detail->mutable_basic()->set_year(year(bytes, length));
  chassis_detail->mutable_basic()->set_month(month(bytes, length));
  chassis_detail->mutable_basic()->set_day(day(bytes, length));
  chassis_detail->mutable_basic()->set_hours(hours(bytes, length));
  chassis_detail->mutable_basic()->set_minutes(minutes(bytes, length));
  chassis_detail->mutable_basic()->set_seconds(seconds(bytes, length));
  chassis_detail->mutable_basic()->set_compass_direction(
      compass_direction(bytes, length));
  chassis_detail->mutable_basic()->set_pdop(pdop(bytes, length));
  chassis_detail->mutable_basic()->set_is_gps_fault(
      is_gps_fault(bytes, length));
  chassis_detail->mutable_basic()->set_is_inferred(
      is_inferred_position(bytes, length));
}

int32_t Gps6e::year(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 0);
  int32_t x = frame.get_byte(0, 7);
  return x;
}

int32_t Gps6e::month(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 1);
  int32_t x = frame.get_byte(0, 4);
  return x;
}

int32_t Gps6e::day(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 2);
  int32_t x = frame.get_byte(0, 5);
  return x;
}

int32_t Gps6e::hours(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 3);
  int32_t x = frame.get_byte(0, 5);
  return x;
}

int32_t Gps6e::minutes(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 4);
  int32_t x = frame.get_byte(0, 6);
  return x;
}

int32_t Gps6e::seconds(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 5);
  int32_t x = frame.get_byte(0, 6);
  return x;
}

double Gps6e::compass_direction(const std::uint8_t *bytes,
                                int32_t length) const {
  Byte frame(bytes + 6);
  int32_t x = frame.get_byte(0, 4);
  return x * 45.000000;
}

double Gps6e::pdop(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 7);
  int32_t x = frame.get_byte(0, 5);
  return x * 0.200000;
}

bool Gps6e::is_gps_fault(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(5);
}

bool Gps6e::is_inferred_position(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(6);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
