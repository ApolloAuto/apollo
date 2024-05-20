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
#include <iomanip>
#include "modules/canbus_vehicle/lincoln/protocol/steering_angle.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t SteeringAngle::ID = 0x0605;

void SteeringAngle::Parse(const std::uint8_t *bytes, int32_t length,
                          Lincoln *chassis_detail) const {
  std::stringstream combined_hex_ss;
  int number1 = static_cast<int>(bytes[4] & 0xFF);
  int number2 = static_cast<int>(bytes[5] & 0xFF);
  int number3 = static_cast<int>(bytes[6] & 0xFF);
  int number4 = static_cast<int>(bytes[7] & 0xFF);
  combined_hex_ss << std::setw(2) << std::setfill('0') << std::hex << number1;
  combined_hex_ss << std::setw(2) << std::setfill('0') << std::hex << number2;
  combined_hex_ss << std::setw(2) << std::setfill('0') << std::hex << number3;
  combined_hex_ss << std::setw(2) << std::setfill('0') << std::hex << number4;
  std::stringstream ss2;
  double angle = 0.0;
  ss2 << std::hex << combined_hex_ss.str();
  unsigned int combined_dec;
  ss2 >> combined_dec;
  int decimal = combined_dec;
  if (combined_dec & (1 << (combined_hex_ss.str().size() * 4 - 1))) {
    decimal -= (1 << (combined_hex_ss.str().size() * 4));
  }

  angle = static_cast<double>(decimal * (360.0 / 14400.0));
  
  chassis_detail->mutable_eps()->set_steering_angle(angle);
  printf(">>>>>>>>方向盘角度: %.2f\n", angle);
}

uint32_t SteeringAngle::GetPeriod() const {
  // receive rate??
  // receive timeout would trigger fault, letting en=0 and etc.
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void SteeringAngle::UpdateData(uint8_t *data) {
  // set_steering_angle_p(data, steering_angle_);
  // set_enable_p(data, steering_enable_);
  // set_clear_driver_override_flag_p(data, clear_driver_override_flag_);
  // set_ignore_driver_override_p(data, ignore_driver_override_);
  // set_steering_angle_speed_p(data, steering_angle_speed_);
  // set_watchdog_counter_p(data, watchdog_counter_);
  // set_disable_audible_warning_p(data, disable_audible_warning_);

  int speed = steering_angle_speed_ * 100;

  for (int i = 0; i < 8; ++i) {
    if (2 == i) {
      data[i] = static_cast<unsigned char>((speed >> 8) & 0xFF);
    } else if (3 == i) {
      data[i] = static_cast<unsigned char>(speed & 0xFF);
    } else if (6 == i) {
      data[i] = static_cast<unsigned char>(1 & 0xFF);
    } else {
      data[i] = static_cast<unsigned char>(0 & 0xFF);
    }
  }
}

void SteeringAngle::Reset() {
  steering_angle_ = 0.0;
  steering_enable_ = false;
  clear_driver_override_flag_ = false;
  ignore_driver_override_ = false;
  steering_angle_speed_ = 0.0;
  watchdog_counter_ = 0;
  disable_audible_warning_ = false;
}

SteeringAngle *SteeringAngle::set_steering_angle(double angle) {
  steering_angle_ = angle;
  return this;
}

SteeringAngle *SteeringAngle::set_enable() {
  steering_enable_ = true;
  return this;
}

SteeringAngle *SteeringAngle::set_disable() {
  steering_enable_ = false;
  return this;
}

SteeringAngle *SteeringAngle::set_steering_angle_speed(double angle_speed) {
  steering_angle_speed_ = angle_speed;
  return this;
}

// private

// positive for left, negative for right
void SteeringAngle::set_steering_angle_p(uint8_t *data, double angle) {
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

void SteeringAngle::set_enable_p(uint8_t *bytes, bool enable) {
  Byte frame(bytes + 2);
  if (enable) {
    frame.set_bit_1(0);
  } else {
    frame.set_bit_0(0);
  }
}

void SteeringAngle::set_clear_driver_override_flag_p(uint8_t *bytes,
                                                     bool clear) {
  Byte frame(bytes + 2);
  if (clear) {
    frame.set_bit_1(1);
  } else {
    frame.set_bit_0(1);
  }
}

void SteeringAngle::set_ignore_driver_override_p(uint8_t *bytes, bool ignore) {
  Byte frame(bytes + 2);
  if (ignore) {
    frame.set_bit_1(2);
  } else {
    frame.set_bit_0(2);
  }
}

void SteeringAngle::set_steering_angle_speed_p(uint8_t *data,
                                               double angle_speed) {
  angle_speed = ProtocolData::BoundedValue(0.0, 500.0, angle_speed);
  int32_t x = static_cast<int32_t>(angle_speed / 2.000000);

  Byte frame(data + 3);
  frame.set_value(static_cast<uint8_t>(x), 0, 8);
}

void SteeringAngle::set_watchdog_counter_p(uint8_t *data, int32_t count) {
  count = ProtocolData::BoundedValue(0, 255, count);
  Byte frame(data + 7);
  frame.set_value(static_cast<uint8_t>(count), 0, 8);
}

void SteeringAngle::set_disable_audible_warning_p(uint8_t *data, bool disable) {
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
