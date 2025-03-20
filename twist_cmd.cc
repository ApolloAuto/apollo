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

#include "modules/serial/vehicle/ros/protocol/twist_cmd.h"

#include "modules/serial/common/util.h"
#include "cyber/common/log.h"

namespace apollo {
namespace serial {

using apollo::drivers::canbus::Byte;

void set_x_target_speed(uint8_t* data, double x_target_speed) {
  x_target_speed = BoundedValue(-5.0, 5.0, x_target_speed);
  //AERROR << "x_target_speed: " << x_target_speed;
  int x = static_cast<int>(x_target_speed * 1000);
  uint8_t t = 0;

  //AERROR << "x: " << x;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte frame_low(data + 4);
  frame_low.set_value(t, 0, 8);
  x >>= 8;

  // AERROR << "t: "  << t;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte frame_high(data + 3);

  // AERROR << "t: " << t;
  frame_high.set_value(t, 0, 8);
}

void set_y_target_speed(uint8_t* data, double y_target_speed) {
  y_target_speed = BoundedValue(-5.0, 5.0, y_target_speed);
  //AERROR << "y_target_speed: " << y_target_speed;
  int y = static_cast<int>(y_target_speed * 1000);
  uint8_t t = 0;

  //AERROR << "y: " << y;

  t = static_cast<uint8_t>(y & 0xFF);
  Byte frame_low(data + 6);
  frame_low.set_value(t, 0, 8);
  y >>= 8;

  // AERROR << "t: "  << t;

  t = static_cast<uint8_t>(y & 0xFF);
  Byte frame_high(data + 5);
  // AERROR << "t: " << t;
  frame_high.set_value(t, 0, 8);
  
}

void set_angular_velocity_z(uint8_t* data, double z_angular_velocity) {
  z_angular_velocity = BoundedValue(-3.0, 3.0, z_angular_velocity);
  int z = static_cast<int>(z_angular_velocity * 1000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(z & 0xFF);
  Byte frame_low(data + 8);
  frame_low.set_value(t, 0, 8);
  z >>= 8;

  t = static_cast<uint8_t>(z & 0xFF);
  Byte frame_high(data + 7);
  frame_high.set_value(t, 0, 8);
}

void set_checksum(uint8_t* data) {
  data[0] = 0x7B;
  data[10] = 0x7D;
  uint8_t checksum = 0;
  for (int i = 0; i < 10; i++) {
    checksum ^= data[i];
  }
  data[9] = checksum;
}

}  // namespace serial
}  // namespace apollo
