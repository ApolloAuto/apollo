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

#include "modules/canbus_vehicle/wey/protocol/fbs1_243.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace wey {

using ::apollo::drivers::canbus::Byte;

Fbs1243::Fbs1243() {}
const int32_t Fbs1243::ID = 0x243;

void Fbs1243::Parse(const std::uint8_t* bytes, int32_t length,
                    Wey* chassis) const {
  chassis->mutable_fbs1_243()->set_longitudeacce(
      longitudeacce(bytes, length));
  chassis->mutable_fbs1_243()->set_lateralacce(
      lateralacce(bytes, length));
  chassis->mutable_fbs1_243()->set_vehdynyawrate(
      vehdynyawrate(bytes, length));
  chassis->mutable_fbs1_243()->set_flwheelspd(
      flwheelspd(bytes, length));
  chassis->mutable_fbs1_243()->set_frwheeldirection(
      frwheeldirection(bytes, length));
}

// config detail: {'description': 'Longitude acceleration', 'offset': -21.592,
// 'precision': 0.00098, 'len': 16, 'name': 'longitudeacce', 'is_signed_var':
// False, 'physical_range': '[-21.592|21.592]', 'bit': 7, 'type': 'double',
// 'order': 'motorola', 'physical_unit': 'm/s^2'}
double Fbs1243::longitudeacce(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.000980 + -21.592000;
  return ret;
}

// config detail: {'description': 'Indicates Lateral Acceleration', 'offset':
// -21.592, 'precision': 0.00098, 'len': 16, 'name': 'lateralacce',
// 'is_signed_var': False, 'physical_range': '[-21.592|21.592]', 'bit': 23,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s^2'}
double Fbs1243::lateralacce(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.000980 + -21.592000;
  return ret;
}

// config detail: {'description': 'Vehicle yaw rate', 'offset': -2.093,
// 'precision': 0.00024, 'len': 16, 'name': 'vehdynyawrate', 'is_signed_var':
// False, 'physical_range': '[-2.093|2.093]', 'bit': 39, 'type': 'double',
// 'order': 'motorola', 'physical_unit': 'rad/s'}
double Fbs1243::vehdynyawrate(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.000240 + -2.093000;
  return ret;
}

// config detail: {'description': 'Front left wheel speed', 'offset': 0.0,
// 'precision': 0.05625, 'len': 13, 'name': 'flwheelspd', 'is_signed_var':
// False, 'physical_range': '[0|299.98125]', 'bit': 55, 'type': 'double',
// 'order': 'motorola', 'physical_unit': 'Km/h'}
double Fbs1243::flwheelspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'description': 'Front right wheel  Moving direction',
// 'enum': {0: 'FRWHEELDIRECTION_INVALID', 1: 'FRWHEELDIRECTION_FORWARD',
// 2: 'FRWHEELDIRECTION_BACKWARD', 3: 'FRWHEELDIRECTION_STOP'}, 'precision':
// 1.0, 'len': 2, 'name': 'frwheeldirection', 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 57, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Fbs1_243::FrwheeldirectionType Fbs1243::frwheeldirection(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 2);

  Fbs1_243::FrwheeldirectionType ret =
      static_cast<Fbs1_243::FrwheeldirectionType>(x);
  return ret;
}
}  // namespace wey
}  // namespace canbus
}  // namespace apollo
