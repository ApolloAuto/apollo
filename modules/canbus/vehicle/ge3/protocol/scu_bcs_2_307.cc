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

#include "modules/canbus/vehicle/ge3/protocol/scu_bcs_2_307.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

Scubcs2307::Scubcs2307() {}
const int32_t Scubcs2307::ID = 0x307;

void Scubcs2307::Parse(const std::uint8_t* bytes, int32_t length,
                       ChassisDetail* chassis) const {
  chassis->mutable_ge3()->mutable_scu_bcs_2_307()->set_bcs_vehspdvd(
      bcs_vehspdvd(bytes, length));
  chassis->mutable_ge3()->mutable_scu_bcs_2_307()->set_bcs_yawrate(
      bcs_yawrate(bytes, length));
  chassis->mutable_ge3()->mutable_scu_bcs_2_307()->set_bcs_vehspd(
      bcs_vehspd(bytes, length));
  chassis->mutable_ge3()->mutable_scu_bcs_2_307()->set_bcs_vehlongaccel(
      bcs_vehlongaccel(bytes, length));
  chassis->mutable_ge3()->mutable_scu_bcs_2_307()->set_bcs_vehlataccel(
      bcs_vehlataccel(bytes, length));
}

// config detail: {'description': 'Vehicle speed valid data', 'enum': {0:
// 'BCS_VEHSPDVD_INVALID', 1: 'BCS_VEHSPDVD_VALID'}, 'precision': 1.0, 'len': 1,
// 'name': 'bcs_vehspdvd', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 40, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': '-'}
Scu_bcs_2_307::Bcs_vehspdvdType Scubcs2307::bcs_vehspdvd(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 1);

  Scu_bcs_2_307::Bcs_vehspdvdType ret =
      static_cast<Scu_bcs_2_307::Bcs_vehspdvdType>(x);
  return ret;
}

// config detail: {'description': 'Yaw rate', 'offset': -2.2243, 'precision':
// 0.0021326, 'len': 12, 'name': 'bcs_yawrate', 'is_signed_var': False,
// 'physical_range': '[-2.2243|2.2243]', 'bit': 55, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'rad/s'}
double Scubcs2307::bcs_yawrate(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.002133 + -2.224300;
  return ret;
}

// config detail: {'description': 'Vehicle speed', 'offset': 0.0, 'precision':
// 0.05625, 'len': 13, 'name': 'bcs_vehspd', 'is_signed_var': False,
// 'physical_range': '[0|240]', 'bit': 39, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'km/h'}
double Scubcs2307::bcs_vehspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret = x * 0.056250 / 3.6;  // modified by 20181211 , change km/h to m/s
  return ret;
}

// config detail: {'description': 'Vehicle longitudinal acceleration', 'offset':
// -21.593, 'precision': 0.027126736, 'len': 12, 'name': 'bcs_vehlongaccel',
// 'is_signed_var': False, 'physical_range': '[-21.593|21.593]', 'bit': 23,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s^2'}
double Scubcs2307::bcs_vehlongaccel(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.027127 + -21.593000;
  return ret;
}

// config detail: {'description': 'Vehicle lateral acceleration', 'offset':
// -21.593, 'precision': 0.027126736, 'len': 12, 'name': 'bcs_vehlataccel',
// 'is_signed_var': False, 'physical_range': '[-21.593|21.593]', 'bit': 7,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s^2'}
double Scubcs2307::bcs_vehlataccel(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.027127 + -21.593000;
  return ret;
}
}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
