// Copyright 2025 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-01-16
//  Author: daohu527

#include "modules/serial/vehicle/ros/protocol/twist_fb.h"

namespace apollo {
namespace serial {

using apollo::drivers::canbus::Byte;

static constexpr double ACCELERATION_SCALE_FACTOR = 1672.0;
static constexpr double ANGULAR_VELOCITY_SCALE_FACTOR = 3753.0;

int flag_stop(const std::uint8_t* bytes, const int32_t length) {
  Byte t1(bytes + 1);
  int32_t x = t1.get_byte(0, 8);
  return x;
}

double x_speed(const std::uint8_t* bytes, const int32_t length) {
  Byte high(bytes + 2);
  int32_t x = high.get_byte(0, 8);

  Byte low(bytes + 3);
  int32_t t = low.get_byte(0, 8);
  x <<= 8;
  x |= t;

  if (high.is_bit_1(7)) {
    x -= 0x10000;
  }
  // mm/s to m/s
  double ret = x * 0.001;
  return ret;
}

double y_speed(const std::uint8_t* bytes, const int32_t length) {
  Byte high(bytes + 4);
  int32_t x = high.get_byte(0, 8);

  Byte low(bytes + 5);
  int32_t t = low.get_byte(0, 8);
  x <<= 8;
  x |= t;

  if (high.is_bit_1(7)) {
    x -= 0x10000;
  }
  // mm/s to m/s
  double ret = x * 0.001;
  return ret;
}

double z_speed(const std::uint8_t* bytes, const int32_t length) {
  Byte high(bytes + 6);
  int32_t x = high.get_byte(0, 8);

  Byte low(bytes + 7);
  int32_t t = low.get_byte(0, 8);
  x <<= 8;
  x |= t;

  if (high.is_bit_1(7)) {
    x -= 0x10000;
  }
  // Counterclockwise is +, clockwise is -
  double ret = x * 0.001;
  return ret;
}

double acceleration_x(const std::uint8_t* bytes, const int32_t length) {
  Byte high(bytes + 8);
  int32_t x = high.get_byte(0, 8);

  Byte low(bytes + 9);
  int32_t t = low.get_byte(0, 8);
  x <<= 8;
  x |= t;

  if (high.is_bit_1(7)) {
    x -= 0x10000;
  }
  // Convert original data to m/s² by dividing by 1672
  return static_cast<double>(x) / ACCELERATION_SCALE_FACTOR;
}

double acceleration_y(const std::uint8_t* bytes, const int32_t length) {
  Byte high(bytes + 10);
  int32_t x = high.get_byte(0, 8);

  Byte low(bytes + 11);
  int32_t t = low.get_byte(0, 8);
  x <<= 8;
  x |= t;

  if (high.is_bit_1(7)) {
    x -= 0x10000;
  }
  // Convert original data to m/s² by dividing by 1672
  return static_cast<double>(x) / ACCELERATION_SCALE_FACTOR;
}

double acceleration_z(const std::uint8_t* bytes, const int32_t length) {
  Byte high(bytes + 12);
  int32_t x = high.get_byte(0, 8);

  Byte low(bytes + 13);
  int32_t t = low.get_byte(0, 8);
  x <<= 8;
  x |= t;

  if (high.is_bit_1(7)) {
    x -= 0x10000;
  }
  // Convert original data to m/s² by dividing by 1672
  return static_cast<double>(x) / ACCELERATION_SCALE_FACTOR;
}

double angular_velocity_x(const std::uint8_t* bytes, const int32_t length) {
  Byte high(bytes + 14);
  int32_t x = high.get_byte(0, 8);

  Byte low(bytes + 15);
  int32_t t = low.get_byte(0, 8);
  x <<= 8;
  x |= t;

  if (high.is_bit_1(7)) {
    x -= 0x10000;
  }
  // Convert original data to rad/s by dividing by 3753
  return static_cast<double>(x) / ANGULAR_VELOCITY_SCALE_FACTOR;
}

double angular_velocity_y(const std::uint8_t* bytes, const int32_t length) {
  Byte high(bytes + 16);
  int32_t x = high.get_byte(0, 8);

  Byte low(bytes + 17);
  int32_t t = low.get_byte(0, 8);
  x <<= 8;
  x |= t;

  if (high.is_bit_1(7)) {
    x -= 0x10000;
  }
  // Convert original data to rad/s by dividing by 3753
  return static_cast<double>(x) / ANGULAR_VELOCITY_SCALE_FACTOR;
}

double angular_velocity_z(const std::uint8_t* bytes, const int32_t length) {
  Byte high(bytes + 18);
  int32_t x = high.get_byte(0, 8);

  Byte low(bytes + 19);
  int32_t t = low.get_byte(0, 8);
  x <<= 8;
  x |= t;

  if (high.is_bit_1(7)) {
    x -= 0x10000;
  }
  // Convert original data to rad/s by dividing by 3753
  return static_cast<double>(x) / ANGULAR_VELOCITY_SCALE_FACTOR;
}

double battery_voltage(const std::uint8_t* bytes, const int32_t length) {
  Byte high(bytes + 20);
  int32_t x = high.get_byte(0, 8);

  Byte low(bytes + 21);
  int32_t t = low.get_byte(0, 8);
  x <<= 8;
  x |= t;

  return static_cast<double>(x);
}

}  // namespace serial
}  // namespace apollo
