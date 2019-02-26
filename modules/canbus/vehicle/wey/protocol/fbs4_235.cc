/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/wey/protocol/fbs4_235.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace wey {

using ::apollo::drivers::canbus::Byte;

Fbs4235::Fbs4235() {}
const int32_t Fbs4235::ID = 0x235;

void Fbs4235::Parse(const std::uint8_t* bytes, int32_t length,
                    ChassisDetail* chassis) const {
  chassis->mutable_wey()->mutable_fbs4_235()->set_steerwheelangle(
      steerwheelangle(bytes, length));
  chassis->mutable_wey()->mutable_fbs4_235()->set_steerwheelspd(
      steerwheelspd(bytes, length));
}

// config detail: {'description': 'angle of steering wheel ',
// 'offset': 0.0, 'precision': 0.1, 'len': 15, 'name': 'steerwheelangle',
// 'is_signed_var': False, 'physical_range': '[0|780]', 'bit': 15,
// 'type': 'double', 'order': 'motorola', 'physical_unit': '\xa1\xe3'}
double Fbs4235::steerwheelangle(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'description': 'steering wheel rotation speed',
// 'offset': 0.0, 'precision': 0.1, 'len': 15, 'name': 'steerwheelspd',
// 'is_signed_var': False, 'physical_range': '[0|1016]', 'bit': 39,
// 'type': 'double', 'order': 'motorola', 'physical_unit': '\xa1\xe3/s'}
double Fbs4235::steerwheelspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}
}  // namespace wey
}  // namespace canbus
}  // namespace apollo
