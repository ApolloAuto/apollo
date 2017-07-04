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

#include "modules/canbus/vehicle/lincoln/protocol/wheelspeed_6a.h"

#include "modules/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

const int32_t Wheelspeed6a::ID = 0x6A;

void Wheelspeed6a::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* car_status) const {
  // TODO

  // how to set direction
  // what is "valid"
  // front left
  car_status->mutable_vehicle_spd()->set_wheel_spd_fl(
      front_left_wheel_speed(bytes, length));
  car_status->mutable_vehicle_spd()->set_is_wheel_spd_fl_valid(true);
  // front right
  car_status->mutable_vehicle_spd()->set_wheel_spd_fr(
      front_right_wheel_speed(bytes, length));
  car_status->mutable_vehicle_spd()->set_is_wheel_spd_fr_valid(true);
  // rear left
  car_status->mutable_vehicle_spd()->set_wheel_spd_rl(
      rear_left_wheel_speed(bytes, length));
  car_status->mutable_vehicle_spd()->set_is_wheel_spd_rl_valid(true);
  // rear right
  car_status->mutable_vehicle_spd()->set_wheel_spd_rr(
      rear_right_wheel_speed(bytes, length));
  car_status->mutable_vehicle_spd()->set_is_wheel_spd_rr_valid(true);
  /*
  -?(rr(bytes, length));
  -?(rl(bytes, length));
  -?(fr(bytes, length));
  -?(fl(bytes, length));*/
}

void Wheelspeed6a::Parse(const std::uint8_t* bytes, int32_t length,
                         const struct timeval& timestamp,
                         ChassisDetail* car_status) const {
  car_status->mutable_vehicle_spd()->set_timestamp_sec(
      timestamp.tv_sec + timestamp.tv_usec / 1000000.0);
  Parse(bytes, length, car_status);
}

double Wheelspeed6a::front_left_wheel_speed(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte high_frame(bytes + 1);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes + 0);
  int32_t low = low_frame.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value * 0.010000;
}

double Wheelspeed6a::front_right_wheel_speed(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte high_frame(bytes + 3);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes + 2);
  int32_t low = low_frame.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value * 0.010000;
}

double Wheelspeed6a::rear_left_wheel_speed(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte high_frame(bytes + 5);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes + 4);
  int32_t low = low_frame.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value * 0.010000;
}

double Wheelspeed6a::rear_right_wheel_speed(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte high_frame(bytes + 7);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(bytes + 6);
  int32_t low = low_frame.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value * 0.010000;
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
