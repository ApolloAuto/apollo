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

#include "modules/canbus_vehicle/lincoln/protocol/steering_65.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Steering65::ID = 0x65;

void Steering65::Parse(const std::uint8_t *bytes, int32_t length,
                       Lincoln *chassis_detail) const {
  chassis_detail->mutable_eps()->set_steering_angle(
      steering_angle(bytes, length));
  // no steering angle speed

  chassis_detail->mutable_eps()->set_steering_angle_cmd(
      reported_steering_angle_cmd(bytes, length));
  // ?
  chassis_detail->mutable_eps()->set_is_steering_angle_valid(true);
  // vehicle speed from steering, kph -> mps
  chassis_detail->mutable_eps()->set_vehicle_speed(
      vehicle_speed(bytes, length) / 3.6);

  // speed, as it has a higher accuracy
  // kph -> mps
  chassis_detail->mutable_vehicle_spd()->set_vehicle_spd(
      vehicle_speed(bytes, length) / 3.6);
  chassis_detail->mutable_vehicle_spd()->set_is_vehicle_spd_valid(true);

  chassis_detail->mutable_eps()->set_epas_torque(epas_torque(bytes, length));
  chassis_detail->mutable_eps()->set_steering_enabled(
      is_enabled(bytes, length));
  chassis_detail->mutable_eps()->set_driver_override(
      is_driver_override(bytes, length));
  chassis_detail->mutable_eps()->set_driver_activity(
      is_driver_activity(bytes, length));
  chassis_detail->mutable_eps()->set_watchdog_fault(
      is_watchdog_counter_fault(bytes, length));
  chassis_detail->mutable_eps()->set_channel_1_fault(
      is_channel_1_fault(bytes, length));
  chassis_detail->mutable_eps()->set_channel_2_fault(
      is_channel_2_fault(bytes, length));
  chassis_detail->mutable_eps()->set_calibration_fault(
      is_calibration_fault(bytes, length));
  chassis_detail->mutable_eps()->set_connector_fault(
      is_connector_fault(bytes, length));
  chassis_detail->mutable_check_response()->set_is_eps_online(
      !is_driver_override(bytes, length));
}

void Steering65::Parse(const std::uint8_t *bytes, int32_t length,
                       const struct timeval &timestamp,
                       Lincoln *chassis_detail) const {
  chassis_detail->mutable_eps()->set_timestamp_65(
      static_cast<double>(timestamp.tv_sec) +
      static_cast<double>(timestamp.tv_usec) / 1000000.0);
  Parse(bytes, length, chassis_detail);
}

double Steering65::steering_angle(const std::uint8_t *bytes,
                                  int32_t length) const {
  Byte frame_high(bytes + 1);
  int32_t high = frame_high.get_byte(0, 8);
  Byte frame_low(bytes + 0);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  if (value > 0x8000) {
    value = value - 0x10000;
  }

  return value * 0.100000;
}

double Steering65::reported_steering_angle_cmd(const std::uint8_t *bytes,
                                               int32_t length) const {
  Byte frame_high(bytes + 3);
  int32_t high = frame_high.get_byte(0, 8);
  Byte frame_low(bytes + 2);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  if (value > 0x8000) {
    value = value - 0x10000;
  }

  return value * 0.100000;
}

double Steering65::vehicle_speed(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame_high(bytes + 5);
  int32_t high = frame_high.get_byte(0, 8);
  Byte frame_low(bytes + 4);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value * 0.010000;
}

double Steering65::epas_torque(const std::uint8_t *bytes,
                               int32_t length) const {
  Byte frame(bytes + 6);
  int32_t x = frame.get_byte(0, 8);
  if (x > 0x7F) {
    x -= 0x100;
  }
  return x * 0.062500;
}

bool Steering65::is_enabled(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(0);
}

bool Steering65::is_driver_override(const std::uint8_t *bytes,
                                    int32_t length) const {
  // Cleared on rising edge of EN bit in command message
  Byte frame(bytes + 7);
  return frame.is_bit_1(1);
}

bool Steering65::is_driver_activity(const std::uint8_t *bytes,
                                    int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(2);
}

bool Steering65::is_watchdog_counter_fault(const std::uint8_t *bytes,
                                           int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(3);
}

bool Steering65::is_channel_1_fault(const std::uint8_t *bytes,
                                    int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(4);
}

bool Steering65::is_channel_2_fault(const std::uint8_t *bytes,
                                    int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(5);
}

bool Steering65::is_calibration_fault(const std::uint8_t *bytes,
                                      int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(6);
}

bool Steering65::is_connector_fault(const std::uint8_t *bytes,
                                    int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(7);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
