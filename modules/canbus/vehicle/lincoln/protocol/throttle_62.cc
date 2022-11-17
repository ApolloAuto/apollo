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

#include "modules/canbus/vehicle/lincoln/protocol/throttle_62.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

// public

const int32_t Throttle62::ID = 0x62;

uint32_t Throttle62::GetPeriod() const {
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Throttle62::UpdateData(uint8_t *data) {
  set_pedal_p(data, pedal_cmd_);
  set_enable_p(data, pedal_enable_);
  set_clear_driver_override_flag_p(data, clear_driver_override_flag_);
  set_ignore_driver_override_p(data, ignore_driver_override_);
  set_watchdog_counter_p(data, watchdog_counter_);
}

void Throttle62::Reset() {
  pedal_cmd_ = 0.0;
  pedal_enable_ = false;
  clear_driver_override_flag_ = false;
  ignore_driver_override_ = false;
  watchdog_counter_ = 0;
}

Throttle62 *Throttle62::set_pedal(double pedal) {
  pedal_cmd_ = pedal;
  return this;
}

Throttle62 *Throttle62::set_enable() {
  pedal_enable_ = true;
  return this;
}

Throttle62 *Throttle62::set_disable() {
  pedal_enable_ = false;
  return this;
}

// private

void Throttle62::set_pedal_p(uint8_t *data, double pedal) {
  // change from [0-100] to [0.00-1.00]
  // and a rough mapping
  pedal /= 100.0;
  pedal = ProtocolData::BoundedValue(0.0, 1.0, pedal);
  int32_t x = static_cast<int32_t>(pedal / 1.52590218966964e-05);
  std::uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte frame_low(data + 0);
  frame_low.set_value(t, 0, 8);

  x >>= 8;
  t = static_cast<uint8_t>(x & 0xFF);
  Byte frame_high(data + 1);
  frame_high.set_value(t, 0, 8);
}

void Throttle62::set_enable_p(uint8_t *bytes, bool enable) {
  Byte frame(bytes + 3);
  if (enable) {
    frame.set_bit_1(0);
  } else {
    frame.set_bit_0(0);
  }
}

void Throttle62::set_clear_driver_override_flag_p(uint8_t *bytes, bool clear) {
  Byte frame(bytes + 3);
  if (clear) {
    frame.set_bit_1(1);
  } else {
    frame.set_bit_0(1);
  }
}

void Throttle62::set_ignore_driver_override_p(uint8_t *bytes, bool ignore) {
  Byte frame(bytes + 3);
  if (ignore) {
    frame.set_bit_1(2);
  } else {
    frame.set_bit_0(2);
  }
}

void Throttle62::set_watchdog_counter_p(uint8_t *data, int32_t count) {
  count = ProtocolData::BoundedValue(0, 255, count);
  Byte frame(data + 7);
  frame.set_value(static_cast<uint8_t>(count), 0, 8);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
