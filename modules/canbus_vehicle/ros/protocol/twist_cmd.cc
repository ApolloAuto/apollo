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

#include "modules/canbus_vehicle/ros/protocol/twist_cmd.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ros {

using ::apollo::drivers::canbus::Byte;

const int32_t TwistCmd::ID = 0x181;

TwistCmd::TwistCmd() { Reset(); }

uint32_t TwistCmd::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void TwistCmd::UpdateData(uint8_t* data) {
  set_p_x_target_speed(data, x_target_speed_);
  set_p_y_target_speed(data, y_target_speed_);
  set_p_angular_velocity_z(data, z_angular_velocity_);

  uint8_t checksum = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5];
  set_checksum(data, checksum);
}

void TwistCmd::Reset() {
  // TODO(All) :  you should check this manually
  x_target_speed_ = 0;
  y_target_speed_ = 0;
  z_angular_velocity_ = 0;
}

void TwistCmd::set_x_target_speed(double x_target_speed) {
  x_target_speed_ = x_target_speed;
}

void TwistCmd::set_y_target_speed(double y_target_speed) {
  y_target_speed_ = y_target_speed;
}

void TwistCmd::set_angular_velocity_z(double z_angular_velocity) {
  z_angular_velocity_ = z_angular_velocity;
}

void TwistCmd::set_p_x_target_speed(uint8_t* data, double x_target_speed) {
  x_target_speed = ProtocolData::BoundedValue(-5.0, 5.0, x_target_speed);
  int x = static_cast<int>(x_target_speed * 1000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte frame_low(data + 1);
  frame_low.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte frame_high(data + 0);
  frame_high.set_value(t, 0, 8);
}

void TwistCmd::set_p_y_target_speed(uint8_t* data, double y_target_speed) {
  y_target_speed = ProtocolData::BoundedValue(-5.0, 5.0, y_target_speed);
  int y = static_cast<int>(y_target_speed * 1000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(y & 0xFF);
  Byte frame_low(data + 3);
  frame_low.set_value(t, 0, 8);
  y >>= 8;

  t = static_cast<uint8_t>(y & 0xFF);
  Byte frame_high(data + 2);
  frame_high.set_value(t, 0, 8);
}

void TwistCmd::set_p_angular_velocity_z(uint8_t* data,
                                        double z_angular_velocity) {
  z_angular_velocity =
      ProtocolData::BoundedValue(-3.0, 3.0, z_angular_velocity);
  int z = static_cast<int>(z_angular_velocity * 1000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(z & 0xFF);
  Byte frame_low(data + 5);
  frame_low.set_value(t, 0, 8);
  z >>= 8;

  t = static_cast<uint8_t>(z & 0xFF);
  Byte frame_high(data + 4);
  frame_high.set_value(t, 0, 8);
}

void TwistCmd::set_checksum(uint8_t* data, uint8_t checksum) {
  // TODO(zero): need to implement
}

}  // namespace ros
}  // namespace canbus
}  // namespace apollo
