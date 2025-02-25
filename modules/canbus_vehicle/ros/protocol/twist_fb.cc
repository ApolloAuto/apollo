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

#include "modules/canbus_vehicle/ros/protocol/twist_fb.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ros {

using ::apollo::drivers::canbus::Byte;

TwistFb::TwistFb() {}

const int32_t TwistFb::ID = 0x101;

void TwistFb::Parse(const std::uint8_t* bytes, int32_t length,
                    Ros* chassis) const {
  chassis->mutable_twist_fb()->set_flag_stop(flag_stop(bytes, length));
  chassis->mutable_twist_fb()->set_x_speed(x_speed(bytes, length));
  chassis->mutable_twist_fb()->set_y_speed(y_speed(bytes, length));
  chassis->mutable_twist_fb()->set_z_angle_speed(z_angle_speed(bytes, length));
}

int TwistFb::flag_stop(const std::uint8_t* bytes, const int32_t length) const {
  Byte t1(bytes + 1);
  int32_t x = t1.get_byte(0, 8);
  return x;
}

double TwistFb::x_speed(const std::uint8_t* bytes, const int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001;
  return ret;
}

double TwistFb::y_speed(const std::uint8_t* bytes, const int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001;
  return ret;
}

double TwistFb::z_angle_speed(const std::uint8_t* bytes, const int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 1000;
  return ret;
}

}  // namespace ros
}  // namespace canbus
}  // namespace apollo
