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

#include "modules/canbus/vehicle/ge3/protocol/scu_vcu_2_313.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

Scuvcu2313::Scuvcu2313() {}
const int32_t Scuvcu2313::ID = 0x313;

void Scuvcu2313::Parse(const std::uint8_t* bytes, int32_t length,
                       ChassisDetail* chassis) const {
  chassis->mutable_ge3()->mutable_scu_vcu_2_313()->set_vcu_torqposmax(
      vcu_torqposmax(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_2_313()->set_vcu_torqnegmax(
      vcu_torqnegmax(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_2_313()->set_vcu_torqact(
      vcu_torqact(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_2_313()->set_vcu_engspd(
      vcu_engspd(bytes, length));
}

// config detail: {'description': 'Max positive torque', 'offset': 0.0,
// 'precision': 1.5, 'len': 11, 'name': 'vcu_torqposmax', 'is_signed_var':
// False, 'physical_range': '[0|3000]', 'bit': 55, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'Nm'}
double Scuvcu2313::vcu_torqposmax(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(5, 3);
  x <<= 3;
  x |= t;

  double ret = x * 1.500000;
  return ret;
}

// config detail: {'description': 'Max negative torque', 'offset': -3000.0,
// 'precision': 1.5, 'len': 11, 'name': 'vcu_torqnegmax', 'is_signed_var':
// False, 'physical_range': '[-3000|0]', 'bit': 39, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'Nm'}
double Scuvcu2313::vcu_torqnegmax(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(5, 3);
  x <<= 3;
  x |= t;

  double ret = x * 1.500000 + -3000.000000;
  return ret;
}

// config detail: {'description': 'Actual torque', 'offset': -3000.0,
// 'precision': 1.5, 'len': 12, 'name': 'vcu_torqact', 'is_signed_var': False,
// 'physical_range': '[-3000|3000]', 'bit': 23, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'Nm'}
double Scuvcu2313::vcu_torqact(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 1.500000 + -3000.000000;
  return ret;
}

// config detail: {'description': 'Engine speed', 'offset': 0.0,
// 'precision': 1.0, 'len': 16, 'name': 'vcu_engspd', 'is_signed_var': False,
// 'physical_range': '[0|65535]', 'bit': 7, 'type': 'int', 'order': 'motorola',
// 'physical_unit': 'rpm'}
int Scuvcu2313::vcu_engspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
