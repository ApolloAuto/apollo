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

#include "modules/canbus/vehicle/lincoln/protocol/brake_61.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Brake61::ID = 0x61;

void Brake61::Parse(const std::uint8_t *bytes, int32_t length,
                    ChassisDetail *chassis_detail) const {
  chassis_detail->mutable_brake()->set_brake_input(pedal_input(bytes, length));
  chassis_detail->mutable_brake()->set_brake_cmd(pedal_cmd(bytes, length));
  chassis_detail->mutable_brake()->set_brake_output(
      pedal_output(bytes, length));
  chassis_detail->mutable_brake()->set_boo_input(boo_input(bytes, length));
  chassis_detail->mutable_brake()->set_boo_cmd(boo_cmd(bytes, length));
  chassis_detail->mutable_brake()->set_boo_output(boo_output(bytes, length));
  chassis_detail->mutable_brake()->set_watchdog_applying_brakes(
      is_watchdog_counter_applying_brakes(bytes, length));
  chassis_detail->mutable_brake()->set_watchdog_source(
      watchdog_counter_source(bytes, length));
  chassis_detail->mutable_brake()->set_brake_enabled(is_enabled(bytes, length));
  chassis_detail->mutable_brake()->set_driver_override(
      is_driver_override(bytes, length));
  chassis_detail->mutable_brake()->set_driver_activity(
      is_driver_activity(bytes, length));
  chassis_detail->mutable_brake()->set_watchdog_fault(
      is_watchdog_counter_fault(bytes, length));
  chassis_detail->mutable_brake()->set_channel_1_fault(
      is_channel_1_fault(bytes, length));
  chassis_detail->mutable_brake()->set_channel_2_fault(
      is_channel_2_fault(bytes, length));
  chassis_detail->mutable_brake()->set_boo_fault(
      is_boo_switch_fault(bytes, length));
  chassis_detail->mutable_brake()->set_connector_fault(
      is_connector_fault(bytes, length));
  chassis_detail->mutable_check_response()->set_is_esp_online(
      !is_driver_override(bytes, length));
}

double Brake61::pedal_input(const std::uint8_t *bytes, int32_t length) const {
  DCHECK_GE(length, 2);
  // Pedal Input from the physical pedal
  return parse_two_frames(bytes[0], bytes[1]);
}

double Brake61::pedal_cmd(const std::uint8_t *bytes, int32_t length) const {
  DCHECK_GE(length, 4);
  // Pedal Command from the command message
  return parse_two_frames(bytes[2], bytes[3]);
}

double Brake61::pedal_output(const std::uint8_t *bytes, int32_t length) const {
  DCHECK_GE(length, 6);
  // Pedal Output is the maximum of PI and PC
  return parse_two_frames(bytes[4], bytes[5]);
}

double Brake61::parse_two_frames(const std::uint8_t low_byte,
                                 const std::uint8_t high_byte) const {
  Byte frame_high(&high_byte);
  int32_t high = frame_high.get_byte(0, 8);
  Byte frame_low(&low_byte);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  // control needs a value in range [0, 100] %
  double output = 100.0 * value * 1.52590218966964e-05;
  output = ProtocolData::BoundedValue(0.0, 100.0, output);
  return output;
}

bool Brake61::boo_input(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 6);
  // seems typo here
  return frame.is_bit_1(0);
}

bool Brake61::boo_cmd(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 6);
  return frame.is_bit_1(1);
}

bool Brake61::boo_output(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 6);
  // seems typo here
  return frame.is_bit_1(2);
}

bool Brake61::is_watchdog_counter_applying_brakes(const std::uint8_t *bytes,
                                                  int32_t length) const {
  Byte frame(bytes + 6);
  return frame.is_bit_1(3);
}

int32_t Brake61::watchdog_counter_source(const std::uint8_t *bytes,
                                         int32_t length) const {
  // see table for status code
  Byte frame(bytes + 6);
  int32_t x = frame.get_byte(4, 4);
  return x;
}

bool Brake61::is_enabled(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(0);
}

bool Brake61::is_driver_override(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(1);
}

bool Brake61::is_driver_activity(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(2);
}

bool Brake61::is_watchdog_counter_fault(const std::uint8_t *bytes,
                                        int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(3);
}

bool Brake61::is_channel_1_fault(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(4);
}

bool Brake61::is_channel_2_fault(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(5);
}

bool Brake61::is_boo_switch_fault(const std::uint8_t *bytes,
                                  int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(6);
}

bool Brake61::is_connector_fault(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(7);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
