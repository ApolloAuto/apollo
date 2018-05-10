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

#include "modules/canbus/vehicle/lincoln/protocol/version_7f.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Version7f::ID = 0x7f;

void Version7f::Parse(const std::uint8_t *bytes, int32_t length,
                      ChassisDetail *chassis_detail) const {
  switch (module_name(bytes, length)) {
    case 0x01:
      chassis_detail->mutable_brake()->set_major_version(
          major_version(bytes, length));
      chassis_detail->mutable_brake()->set_minor_version(
          minor_version(bytes, length));
      chassis_detail->mutable_brake()->set_build_number(
          build_number(bytes, length));
      break;
    case 0x02:
      chassis_detail->mutable_gas()->set_major_version(
          major_version(bytes, length));
      chassis_detail->mutable_gas()->set_minor_version(
          minor_version(bytes, length));
      chassis_detail->mutable_gas()->set_build_number(
          build_number(bytes, length));
      break;
    case 0x03:
      chassis_detail->mutable_eps()->set_major_version(
          major_version(bytes, length));
      chassis_detail->mutable_eps()->set_minor_version(
          minor_version(bytes, length));
      chassis_detail->mutable_eps()->set_build_number(
          build_number(bytes, length));
      break;
    default:
      break;
  }
}

int32_t Version7f::module_name(const std::uint8_t *bytes,
                               int32_t length) const {
  Byte frame(bytes + 0);
  int32_t x = frame.get_byte(0, 8);
  return x;  // 0x03 means Steering/Shifter, otherwise ignore
}

int32_t Version7f::major_version(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame_high(bytes + 3);
  int32_t high = frame_high.get_byte(0, 8);
  Byte frame_low(bytes + 2);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value;
}

int32_t Version7f::minor_version(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame_high(bytes + 5);
  int32_t high = frame_high.get_byte(0, 8);
  Byte frame_low(bytes + 4);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value;
}

int32_t Version7f::build_number(const std::uint8_t *bytes,
                                int32_t length) const {
  Byte frame_high(bytes + 7);
  int32_t high = frame_high.get_byte(0, 8);
  Byte frame_low(bytes + 6);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value;
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
