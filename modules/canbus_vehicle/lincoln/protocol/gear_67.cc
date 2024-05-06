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

#include "modules/canbus_vehicle/lincoln/protocol/gear_67.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Gear67::ID = 0x67;

void Gear67::Parse(const std::uint8_t *bytes, int32_t length,
                   Lincoln *chassis_detail) const {
  int32_t gear = gear_state(bytes, length);
  switch (gear) {
    case 0x01:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_PARKING);
      break;
    case 0x02:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_REVERSE);
      break;
    case 0x03:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_NEUTRAL);
      break;
    case 0x04:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_DRIVE);
      break;
    case 0x05:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_LOW);
      break;
    case 0x00:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_NONE);
      break;
    default:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_INVALID);
      break;
  }

  if (is_driver_override(bytes, length)) {
    // last shift requested by driver
    chassis_detail->mutable_gear()->set_is_shift_position_valid(false);
  } else {
    // last shift requested by-wire
    chassis_detail->mutable_gear()->set_is_shift_position_valid(true);
  }
  chassis_detail->mutable_gear()->set_driver_override(
      is_driver_override(bytes, length));

  int32_t gear_cmd = reported_gear_cmd(bytes, length);
  switch (gear_cmd) {
    case 0x01:
      chassis_detail->mutable_gear()->set_gear_cmd(Chassis::GEAR_PARKING);
      break;
    case 0x02:
      chassis_detail->mutable_gear()->set_gear_cmd(Chassis::GEAR_REVERSE);
      break;
    case 0x03:
      chassis_detail->mutable_gear()->set_gear_cmd(Chassis::GEAR_NEUTRAL);
      break;
    case 0x04:
      chassis_detail->mutable_gear()->set_gear_cmd(Chassis::GEAR_DRIVE);
      break;
    case 0x05:
      chassis_detail->mutable_gear()->set_gear_cmd(Chassis::GEAR_LOW);
      break;
    case 0x00:
      chassis_detail->mutable_gear()->set_gear_cmd(Chassis::GEAR_NONE);
      break;
    default:
      chassis_detail->mutable_gear()->set_gear_cmd(Chassis::GEAR_INVALID);
      break;
  }

  chassis_detail->mutable_gear()->set_canbus_fault(
      is_canbus_fault(bytes, length));
}

int32_t Gear67::gear_state(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 0);
  int32_t x = frame.get_byte(0, 3);
  return x;
}

bool Gear67::is_driver_override(const std::uint8_t *bytes,
                                int32_t length) const {
  Byte frame(bytes + 0);
  return frame.is_bit_1(3);
}

int32_t Gear67::reported_gear_cmd(const std::uint8_t *bytes,
                                  int32_t length) const {
  Byte frame(bytes + 0);
  int32_t x = frame.get_byte(4, 3);
  return x;
}

bool Gear67::is_canbus_fault(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 0);
  return frame.is_bit_1(7);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
