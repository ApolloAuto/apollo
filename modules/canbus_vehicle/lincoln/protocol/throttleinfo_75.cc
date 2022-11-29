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

#include "modules/canbus_vehicle/lincoln/protocol/throttleinfo_75.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Throttleinfo75::ID = 0x75;

void Throttleinfo75::Parse(const std::uint8_t *bytes, int32_t length,
                           Lincoln *chassis_detail) const {
  chassis_detail->mutable_ems()->set_engine_rpm(engine_rpm(bytes, length));
  chassis_detail->mutable_gas()->set_accelerator_pedal(
      acc_pedal_percent(bytes, length));
  chassis_detail->mutable_gas()->set_accelerator_pedal_rate(
      acc_pedal_rate(bytes, length));
}

double Throttleinfo75::engine_rpm(const std::uint8_t *bytes,
                                  int32_t length) const {
  Byte frame_high(bytes + 1);
  int32_t high = frame_high.get_byte(0, 8);
  Byte frame_low(bytes + 0);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value * 0.25;
}

double Throttleinfo75::acc_pedal_percent(const std::uint8_t *bytes,
                                         int32_t length) const {
  Byte frame_high(bytes + 3);
  int32_t high = frame_high.get_byte(0, 2);
  Byte frame_low(bytes + 2);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value * 0.1;
}

double Throttleinfo75::acc_pedal_rate(const std::uint8_t *bytes,
                                      int32_t length) const {
  Byte frame(bytes + 4);
  int32_t x = frame.get_byte(0, 8);
  if (x > 0x3F) {
    x -= 0x100;
  }
  return x * 0.04;
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
