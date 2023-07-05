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

#include "modules/canbus_vehicle/lincoln/protocol/wheelspeed_6a.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Wheelspeed6a::ID = 0x6A;

void Wheelspeed6a::Parse(const std::uint8_t *bytes, int32_t length,
                         Lincoln *chassis_detail) const {
  // how to set direction
  // what is "valid"
  // front left
  chassis_detail->mutable_vehicle_spd()->set_wheel_spd_fl(
      front_left_wheel_speed(bytes, length));
  chassis_detail->mutable_vehicle_spd()->set_is_wheel_spd_fl_valid(true);
  chassis_detail->mutable_vehicle_spd()->set_wheel_direction_fl(
      wheel_direction_convert(front_left_wheel_speed(bytes, length)));
  // front right
  chassis_detail->mutable_vehicle_spd()->set_wheel_spd_fr(
      front_right_wheel_speed(bytes, length));
  chassis_detail->mutable_vehicle_spd()->set_is_wheel_spd_fr_valid(true);
  chassis_detail->mutable_vehicle_spd()->set_wheel_direction_fr(
      wheel_direction_convert(front_right_wheel_speed(bytes, length)));
  // rear left
  chassis_detail->mutable_vehicle_spd()->set_wheel_spd_rl(
      rear_left_wheel_speed(bytes, length));
  chassis_detail->mutable_vehicle_spd()->set_is_wheel_spd_rl_valid(true);
  chassis_detail->mutable_vehicle_spd()->set_wheel_direction_rl(
      wheel_direction_convert(rear_left_wheel_speed(bytes, length)));
  // rear right
  chassis_detail->mutable_vehicle_spd()->set_wheel_spd_rr(
      rear_right_wheel_speed(bytes, length));
  chassis_detail->mutable_vehicle_spd()->set_is_wheel_spd_rr_valid(true);
  chassis_detail->mutable_vehicle_spd()->set_wheel_direction_rr(
      wheel_direction_convert(rear_right_wheel_speed(bytes, length)));
  /*
  -?(rr(bytes, length));
  -?(rl(bytes, length));
  -?(fr(bytes, length));
  -?(fl(bytes, length));*/
}

void Wheelspeed6a::Parse(const std::uint8_t *bytes, int32_t length,
                         const struct timeval &timestamp,
                         Lincoln *chassis_detail) const {
  chassis_detail->mutable_vehicle_spd()->set_timestamp_sec(
      static_cast<double>(timestamp.tv_sec) +
      static_cast<double>(timestamp.tv_usec) / 1000000.0);
  Parse(bytes, length, chassis_detail);
}

double Wheelspeed6a::front_left_wheel_speed(const std::uint8_t *bytes,
                                            int32_t length) const {
  DCHECK_GE(length, 2);
  return parse_two_frames(bytes[0], bytes[1]);
}

double Wheelspeed6a::front_right_wheel_speed(const std::uint8_t *bytes,
                                             int32_t length) const {
  DCHECK_GE(length, 4);
  return parse_two_frames(bytes[2], bytes[3]);
}

double Wheelspeed6a::rear_left_wheel_speed(const std::uint8_t *bytes,
                                           int32_t length) const {
  DCHECK_GE(length, 6);
  return parse_two_frames(bytes[4], bytes[5]);
}

double Wheelspeed6a::rear_right_wheel_speed(const std::uint8_t *bytes,
                                            int32_t length) const {
  DCHECK_GE(length, 8);
  return parse_two_frames(bytes[6], bytes[7]);
}

double Wheelspeed6a::parse_two_frames(const std::uint8_t low_byte,
                                      const std::uint8_t high_byte) const {
  Byte high_frame(&high_byte);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(&low_byte);
  int32_t low = low_frame.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  if (value > 0x7FFF) {
    value -= 0x10000;
  }
  return value * 0.010000;
}

WheelSpeed::WheelSpeedType Wheelspeed6a::wheel_direction_convert(
    double wheel_speed) const {
  WheelSpeed::WheelSpeedType wheel_direction = WheelSpeed::INVALID;
  if (wheel_speed > 0) {
    wheel_direction = WheelSpeed::FORWARD;
  } else if ((wheel_speed > -0.01) && (wheel_speed < 0.01)) {
    wheel_direction = WheelSpeed::STANDSTILL;
  } else if (wheel_speed < 0) {
    wheel_direction = WheelSpeed::BACKWARD;
  } else {
    wheel_direction = WheelSpeed::INVALID;
  }
  return wheel_direction;
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
