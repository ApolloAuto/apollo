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

#include "modules/canbus_vehicle/lincoln/protocol/steering_64.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Steering64::ID = 0x64;

uint32_t Steering64::GetPeriod() const {
  // receive rate??
  // receive timeout would trigger fault, letting en=0 and etc.
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Steering64::UpdateData(uint8_t *data) {
  set_steering_angle_p(data, steering_angle_);
  set_enable_p(data, steering_enable_);
  set_clear_driver_override_flag_p(data, clear_driver_override_flag_);
  set_ignore_driver_override_p(data, ignore_driver_override_);
  set_steering_angle_speed_p(data, steering_angle_speed_);
  set_watchdog_counter_p(data, watchdog_counter_);
  set_disable_audible_warning_p(data, disable_audible_warning_);
}

void Steering64::Reset() {
  steering_angle_ = 0.0;
  steering_enable_ = false;
  clear_driver_override_flag_ = false;
  ignore_driver_override_ = false;
  steering_angle_speed_ = 0.0;
  watchdog_counter_ = 0;
  disable_audible_warning_ = false;
}

Steering64 *Steering64::set_steering_angle(double angle) {
  steering_angle_ = angle;
  return this;
}

Steering64 *Steering64::set_enable() {
  steering_enable_ = true;
  return this;
}

Steering64 *Steering64::set_disable() {
  steering_enable_ = false;
  return this;
}

Steering64 *Steering64::set_steering_angle_speed(double angle_speed) {
  steering_angle_speed_ = angle_speed;
  return this;
}

// private

// positive for left, negative for right
void Steering64::set_steering_angle_p(uint8_t *data, double angle) {
  angle = ProtocolData::BoundedValue(-470.0, 470.0, angle);
  int32_t x = static_cast<int32_t>(angle / 0.100000);

  // add offset
  if (x < 0) {
    x += 0x10000;
  }

  std::uint8_t t = 0;
  t = static_cast<uint8_t>(x & 0xFF);  // low
  Byte frame_low(data + 0);
  frame_low.set_value(t, 0, 8);

  x >>= 8;  // high
  t = static_cast<uint8_t>(x & 0xFF);
  Byte frame_high(data + 1);
  frame_high.set_value(t, 0, 8);
}

void Steering64::set_enable_p(uint8_t *bytes, bool enable) {
  Byte frame(bytes + 2);
  if (enable) {
    frame.set_bit_1(0);
  } else {
    frame.set_bit_0(0);
  }
}

void Steering64::set_clear_driver_override_flag_p(uint8_t *bytes, bool clear) {
  Byte frame(bytes + 2);
  if (clear) {
    frame.set_bit_1(1);
  } else {
    frame.set_bit_0(1);
  }
}

void Steering64::set_ignore_driver_override_p(uint8_t *bytes, bool ignore) {
  Byte frame(bytes + 2);
  if (ignore) {
    frame.set_bit_1(2);
  } else {
    frame.set_bit_0(2);
  }
}

void Steering64::set_steering_angle_speed_p(uint8_t *data, double angle_speed) {
  angle_speed = ProtocolData::BoundedValue(0.0, 500.0, angle_speed);
  int32_t x = static_cast<int32_t>(angle_speed / 2.000000);

  Byte frame(data + 3);
  frame.set_value(static_cast<uint8_t>(x), 0, 8);
}

void Steering64::set_watchdog_counter_p(uint8_t *data, int32_t count) {
  count = ProtocolData::BoundedValue(0, 255, count);
  Byte frame(data + 7);
  frame.set_value(static_cast<uint8_t>(count), 0, 8);
}

void Steering64::set_disable_audible_warning_p(uint8_t *data, bool disable) {
  Byte frame(data + 2);
  if (disable) {
    frame.set_bit_1(4);
  } else {
    frame.set_bit_0(4);
  }
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
