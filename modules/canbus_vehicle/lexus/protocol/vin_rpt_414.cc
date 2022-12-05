/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/lexus/protocol/vin_rpt_414.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Vinrpt414::Vinrpt414() {}
const int32_t Vinrpt414::ID = 0x414;

void Vinrpt414::Parse(const std::uint8_t* bytes, int32_t length,
                      Lexus* chassis) const {
  chassis->mutable_vin_rpt_414()->set_veh_serial(
      veh_serial(bytes, length));
  chassis->mutable_vin_rpt_414()->set_veh_my_code(
      veh_my_code(bytes, length));
  chassis->mutable_vin_rpt_414()->set_veh_mfg_code(
      veh_mfg_code(bytes, length));
}

// config detail: {'name': 'veh_serial', 'offset': 0.0, 'precision': 1.0, 'len':
// 24, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39, 'type':
// 'int', 'order': 'motorola', 'physical_unit': ''}
int Vinrpt414::veh_serial(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 6);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'veh_my_code', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 31,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vinrpt414::veh_my_code(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'veh_mfg_code', 'offset': 0.0, 'precision': 1.0,
// 'len': 24, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vinrpt414::veh_mfg_code(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 2);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
