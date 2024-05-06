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

#include "modules/canbus_vehicle/ge3/protocol/scu_eps_311.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

Scueps311::Scueps311() {}
const int32_t Scueps311::ID = 0x311;

void Scueps311::Parse(const std::uint8_t* bytes, int32_t length,
                      Ge3* chassis) const {
  chassis->mutable_scu_eps_311()->set_eps_intidx(
      eps_intidx(bytes, length));
  chassis->mutable_scu_eps_311()->set_eps_steeranglespd(
      eps_steeranglespd(bytes, length));
  chassis->mutable_scu_eps_311()->set_eps_steerangle(
      eps_steerangle(bytes, length));
  chassis->mutable_scu_eps_311()->set_eps_faultst(
      eps_faultst(bytes, length));
  chassis->mutable_scu_eps_311()->set_eps_drvmode(
      eps_drvmode(bytes, length));
}

// config detail: {'description': 'EPS interrupt index', 'enum': {0:
// 'EPS_INTIDX_NOINT', 1: 'EPS_INTIDX_OVERFLOW', 2: 'EPS_INTIDX_TIMEOUT', 3:
// 'EPS_INTIDX_STEERINT'}, 'precision': 1.0, 'len': 3, 'name': 'eps_intidx',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 6,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Scu_eps_311::Eps_intidxType Scueps311::eps_intidx(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 3);

  Scu_eps_311::Eps_intidxType ret = static_cast<Scu_eps_311::Eps_intidxType>(x);
  return ret;
}

// config detail: {'description': 'Steer angle speed', 'offset': 0.0,
// 'precision': 4.0, 'len': 8, 'name': 'eps_steeranglespd', 'is_signed_var':
// False, 'physical_range': '[0|1016]', 'bit': 15, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'deg/s'}
double Scueps311::eps_steeranglespd(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 4.000000;
  return ret;
}

// config detail: {'description': 'Steer angle Left + right -', 'offset':
// -780.0, 'precision': 0.1, 'len': 16, 'name': 'eps_steerangle',
// 'is_signed_var': False, 'physical_range': '[-780|779.9]', 'bit': 23, 'type':
// 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Scueps311::eps_steerangle(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -780.000000;
  return ret;
}

// config detail: {'description': 'EPS fault status', 'enum': {0:
// 'EPS_FAULTST_NORMAL', 1: 'EPS_FAULTST_FAULT'}, 'precision': 1.0, 'len': 1,
// 'name': 'eps_faultst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_eps_311::Eps_faultstType Scueps311::eps_faultst(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  Scu_eps_311::Eps_faultstType ret =
      static_cast<Scu_eps_311::Eps_faultstType>(x);
  return ret;
}

// config detail: {'description': 'EPS drive mode', 'enum': {0:
// 'EPS_DRVMODE_INVALID', 1: 'EPS_DRVMODE_MANUAL', 2: 'EPS_DRVMODE_INTERRUPT',
// 3: 'EPS_DRVMODE_AUTO'}, 'precision': 1.0, 'len': 2, 'name': 'eps_drvmode',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Scu_eps_311::Eps_drvmodeType Scueps311::eps_drvmode(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  Scu_eps_311::Eps_drvmodeType ret =
      static_cast<Scu_eps_311::Eps_drvmodeType>(x);
  return ret;
}
}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
