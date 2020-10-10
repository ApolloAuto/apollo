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

#include "modules/canbus/vehicle/wey/protocol/fbs2_240.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace wey {

using ::apollo::drivers::canbus::Byte;

Fbs2240::Fbs2240() {}
const int32_t Fbs2240::ID = 0x240;

void Fbs2240::Parse(const std::uint8_t* bytes, int32_t length,
                    ChassisDetail* chassis) const {
  chassis->mutable_wey()->mutable_fbs2_240()->set_flwheeldirection(
      flwheeldirection(bytes, length));
  chassis->mutable_wey()->mutable_fbs2_240()->set_frwheelspd(
      frwheelspd(bytes, length));
  chassis->mutable_wey()->mutable_fbs2_240()->set_rlwheeldrivedirection(
      rlwheeldrivedirection(bytes, length));
  chassis->mutable_wey()->mutable_fbs2_240()->set_rlwheelspd(
      rlwheelspd(bytes, length));
  chassis->mutable_wey()->mutable_fbs2_240()->set_rrwheeldirection(
      rrwheeldirection(bytes, length));
  chassis->mutable_wey()->mutable_fbs2_240()->set_rrwheelspd(
      rrwheelspd(bytes, length));
  // change km/h to m/s
  chassis->mutable_wey()->mutable_fbs2_240()->set_vehiclespd(
      vehiclespd(bytes, length) / 3.6);
}

// config detail: {'description': 'Front left wheel Moving direction',
// 'enum': {0: 'FLWHEELDIRECTION_INVALID', 1: 'FLWHEELDIRECTION_FORWARD',
// 2: 'FLWHEELDIRECTION_BACKWARD', 3: 'FLWHEELDIRECTION_STOP'},
// 'precision': 1.0, 'len': 2, 'name': 'flwheeldirection',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]',
// 'bit': 57, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Fbs2_240::FlwheeldirectionType Fbs2240::flwheeldirection(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 2);

  Fbs2_240::FlwheeldirectionType ret =
      static_cast<Fbs2_240::FlwheeldirectionType>(x);
  return ret;
}

// config detail: {'description': 'Front right wheel speed', 'offset': 0.0,
// 'precision': 0.05625, 'len': 13, 'name': 'frwheelspd',
// 'is_signed_var': False, 'physical_range': '[0|299.98125]', 'bit': 7,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'Km/h'}
double Fbs2240::frwheelspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'description': 'Rear left wheel  Moving direction', 'enum':
// {0: 'RLWHEELDRIVEDIRECTION_INVALID', 1: 'RLWHEELDRIVEDIRECTION_FORWARD',
// 2: 'RLWHEELDRIVEDIRECTION_BACKWARD', 3: 'RLWHEELDRIVEDIRECTION_STOP'},
// 'precision': 1.0, 'len': 2, 'name': 'rlwheeldrivedirection',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]',
// 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Fbs2_240::RlwheeldrivedirectionType Fbs2240::rlwheeldrivedirection(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 2);

  Fbs2_240::RlwheeldrivedirectionType ret =
      static_cast<Fbs2_240::RlwheeldrivedirectionType>(x);
  return ret;
}

// config detail: {'description': 'Rear left wheel speed', 'offset': 0.0,
// 'precision': 0.05625, 'len': 13, 'name':'rlwheelspd','is_signed_var': False,
// 'physical_range': '[0|299.98125]', 'bit': 23, 'type': 'double',
// 'order': 'motorola', 'physical_unit': 'Km/h'}
double Fbs2240::rlwheelspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'description': 'Rear right wheel Moving direction', 'enum':
// {0: 'RRWHEELDIRECTION_INVALID', 1: 'RRWHEELDIRECTION_FORWARD',
// 2: 'RRWHEELDIRECTION_BACKWARD', 3: 'RRWHEELDIRECTION_STOP'},'precision':1.0,
// 'len': 2, 'name': 'rrwheeldirection', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|3]', 'bit': 25, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Fbs2_240::RrwheeldirectionType Fbs2240::rrwheeldirection(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 2);

  Fbs2_240::RrwheeldirectionType ret =
      static_cast<Fbs2_240::RrwheeldirectionType>(x);
  return ret;
}

// config detail: {'description': 'Rear right wheel speed', 'offset': 0.0,
// 'precision': 0.05625, 'len': 13, 'name': 'rrwheelspd','is_signed_var':False,
// 'physical_range': '[0|299.98125]', 'bit': 39, 'type': 'double',
// 'order': 'motorola', 'physical_unit': 'Km/h'}
double Fbs2240::rrwheelspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'description': 'Current Vehicle speed information','offset':
// 0.0, 'precision': 0.05625, 'len': 13, 'name': 'vehiclespd',
// 'is_signed_var': False, 'physical_range': '[0|299.98125]', 'bit': 55,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'Km/h'}
double Fbs2240::vehiclespd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}
}  // namespace wey
}  // namespace canbus
}  // namespace apollo
