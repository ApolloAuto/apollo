// Copyright 2025 daohu527@gmail.com
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

#include "modules/canbus_vehicle/ros/protocol/acc_fb.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ros {

using ::apollo::drivers::canbus::Byte;

AccFb::AccFb() {}

const int32_t AccFb::ID = 0x102;
static constexpr double ACCELERATION_SCALE_FACTOR = 1672.0;
static constexpr double ANGULAR_VELOCITY_SCALE_FACTOR = 3753.0;

void AccFb::Parse(const std::uint8_t* bytes, int32_t length,
                  Ros* chassis) const {
  chassis->mutable_acc_fb()->set_acceleration_x(acceleration_x(bytes, length));
  chassis->mutable_acc_fb()->set_acceleration_y(acceleration_y(bytes, length));
  chassis->mutable_acc_fb()->set_acceleration_z(acceleration_z(bytes, length));
  chassis->mutable_acc_fb()->set_angular_velocity_x(
      angular_velocity_x(bytes, length));
}

double AccFb::acceleration_x(const std::uint8_t* bytes,
                             const int32_t length) const {
  Byte high(bytes + 0);
  int32_t x = high.get_byte(0, 8);

  Byte low(bytes + 1);
  int32_t t = low.get_byte(0, 8);
  x <<= 8;
  x |= t;

  if (high.is_bit_1(7)) {
    x -= 0x10000;
  }
  // Convert original data to m/s² by dividing by 1672
  return static_cast<double>(x) / ACCELERATION_SCALE_FACTOR;
}

double AccFb::acceleration_y(const std::uint8_t* bytes,
                             const int32_t length) const {
  Byte high(bytes + 2);
  int32_t x = high.get_byte(0, 8);

  Byte low(bytes + 3);
  int32_t t = low.get_byte(0, 8);
  x <<= 8;
  x |= t;

  if (high.is_bit_1(7)) {
    x -= 0x10000;
  }
  // Convert original data to m/s² by dividing by 1672
  return static_cast<double>(x) / ACCELERATION_SCALE_FACTOR;
}

double AccFb::acceleration_z(const std::uint8_t* bytes,
                             const int32_t length) const {
  Byte high(bytes + 4);
  int32_t x = high.get_byte(0, 8);

  Byte low(bytes + 5);
  int32_t t = low.get_byte(0, 8);
  x <<= 8;
  x |= t;

  if (high.is_bit_1(7)) {
    x -= 0x10000;
  }
  // Convert original data to m/s² by dividing by 1672
  return static_cast<double>(x) / ACCELERATION_SCALE_FACTOR;
}

double AccFb::angular_velocity_x(const std::uint8_t* bytes,
                                 const int32_t length) const {
  Byte high(bytes + 6);
  int32_t x = high.get_byte(0, 8);

  Byte low(bytes + 7);
  int32_t t = low.get_byte(0, 8);
  x <<= 8;
  x |= t;

  if (high.is_bit_1(7)) {
    x -= 0x10000;
  }
  // Convert original data to rad/s by dividing by 3753
  return static_cast<double>(x) / ANGULAR_VELOCITY_SCALE_FACTOR;
}

}  // namespace ros
}  // namespace canbus
}  // namespace apollo
