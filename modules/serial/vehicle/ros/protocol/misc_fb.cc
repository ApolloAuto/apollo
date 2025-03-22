// Copyright 2025 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-01-16
//  Author: daohu527

#include "modules/serial/vehicle/ros/protocol/misc_fb.h"

namespace apollo {
namespace serial {

using apollo::drivers::canbus::Byte;

double ultrasound(const std::uint8_t* bytes, const int32_t length) {
  Byte high(bytes + 2);
  int32_t x = high.get_byte(0, 8);

  Byte low(bytes + 3);
  int32_t t = low.get_byte(0, 8);
  x <<= 8;
  x |= t;

  if (high.is_bit_1(7)) {
    x -= 0x10000;
  }
  // mm/s to m/s
  double ret = x * 0.001;
  return ret;
}

void checksum(const std::uint8_t* bytes, const int32_t length) {
  // TODO(zero): implement the checksum function
}

}  // namespace serial
}  // namespace apollo
