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

#include "modules/canbus_vehicle/lincoln/protocol/throttle_63.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Throttle63::ID = 0x63;

void Throttle63::Parse(const std::uint8_t *bytes, int32_t length,
                       Lincoln *chassis_detail) const {
  chassis_detail->mutable_gas()->set_throttle_input(pedal_input(bytes, length));
  chassis_detail->mutable_gas()->set_throttle_cmd(pedal_cmd(bytes, length));
  chassis_detail->mutable_gas()->set_throttle_output(
      pedal_output(bytes, length));
  chassis_detail->mutable_gas()->set_watchdog_source(
      watchdog_counter_source(bytes, length));
  chassis_detail->mutable_gas()->set_throttle_enabled(
      is_enabled(bytes, length));
  chassis_detail->mutable_gas()->set_driver_override(
      is_driver_override(bytes, length));
  chassis_detail->mutable_gas()->set_driver_activity(
      is_driver_activity(bytes, length));
  chassis_detail->mutable_gas()->set_watchdog_fault(
      is_watchdog_counter_fault(bytes, length));
  chassis_detail->mutable_gas()->set_channel_1_fault(
      is_channel_1_fault(bytes, length));
  chassis_detail->mutable_gas()->set_channel_2_fault(
      is_channel_2_fault(bytes, length));
  chassis_detail->mutable_gas()->set_connector_fault(
      is_connector_fault(bytes, length));
  chassis_detail->mutable_check_response()->set_is_vcu_online(
      !is_driver_override(bytes, length));
}

double Throttle63::pedal_input(const std::uint8_t *bytes,
                               int32_t length) const {
  // Pedal Input from the physical pedal
  Byte frame_high(bytes + 1);
  int32_t high = frame_high.get_byte(0, 8);
  Byte frame_low(bytes + 0);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  // control needs a value in range [0, 100] %
  double output = 100.0 * value * 1.52590218966964e-05;
  output = ProtocolData::BoundedValue(0.0, 100.0, output);
  return output;
}

double Throttle63::pedal_cmd(const std::uint8_t *bytes, int32_t length) const {
  // Pedal Command from the command message
  Byte frame_high(bytes + 3);
  int32_t high = frame_high.get_byte(0, 8);
  Byte frame_low(bytes + 2);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  // control needs a value in range [0, 100] %
  double output = 100.0 * value * 1.52590218966964e-05;
  output = ProtocolData::BoundedValue(0.0, 100.0, output);
  return output;
}

double Throttle63::pedal_output(const std::uint8_t *bytes,
                                int32_t length) const {
  // Pedal Output is the maximum of PI and PC
  Byte frame_high(bytes + 5);
  int32_t high = frame_high.get_byte(0, 8);
  Byte frame_low(bytes + 4);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  // control needs a value in range [0, 100] %
  double output = 100.0 * value * 1.52590218966964e-05;
  output = ProtocolData::BoundedValue(0.0, 100.0, output);
  return output;
}

int32_t Throttle63::watchdog_counter_source(const std::uint8_t *bytes,
                                            int32_t length) const {
  Byte frame(bytes + 6);
  int32_t x = frame.get_byte(4, 4);
  return x;
}

bool Throttle63::is_enabled(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(0);
}

bool Throttle63::is_driver_override(const std::uint8_t *bytes,
                                    int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(1);
}

bool Throttle63::is_driver_activity(const std::uint8_t *bytes,
                                    int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(2);
}

bool Throttle63::is_watchdog_counter_fault(const std::uint8_t *bytes,
                                           int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(3);
}

bool Throttle63::is_channel_1_fault(const std::uint8_t *bytes,
                                    int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(4);
}

bool Throttle63::is_channel_2_fault(const std::uint8_t *bytes,
                                    int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(5);
}

bool Throttle63::is_connector_fault(const std::uint8_t *bytes,
                                    int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(7);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
