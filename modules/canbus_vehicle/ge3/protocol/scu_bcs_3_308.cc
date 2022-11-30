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

#include "modules/canbus_vehicle/ge3/protocol/scu_bcs_3_308.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

Scubcs3308::Scubcs3308() {}
const int32_t Scubcs3308::ID = 0x308;

void Scubcs3308::Parse(const std::uint8_t* bytes, int32_t length,
                       Ge3* chassis) const {
  chassis->mutable_scu_bcs_3_308()->set_bcs_rrwheelspdvd(
      bcs_rrwheelspdvd(bytes, length));
  chassis->mutable_scu_bcs_3_308()->set_bcs_rrwheeldirectionvd(
      bcs_rrwheeldirectionvd(bytes, length));
  chassis->mutable_scu_bcs_3_308()->set_bcs_rlwheelspdvd(
      bcs_rlwheelspdvd(bytes, length));
  chassis->mutable_scu_bcs_3_308()->set_bcs_rlwheeldirectionvd(
      bcs_rlwheeldirectionvd(bytes, length));
  chassis->mutable_scu_bcs_3_308()->set_bcs_frwheelspdvd(
      bcs_frwheelspdvd(bytes, length));
  chassis->mutable_scu_bcs_3_308()->set_bcs_frwheeldirectionvd(
      bcs_frwheeldirectionvd(bytes, length));
  chassis->mutable_scu_bcs_3_308()->set_bcs_flwheelspdvd(
      bcs_flwheelspdvd(bytes, length));
  chassis->mutable_scu_bcs_3_308()->set_bcs_flwheeldirectionvd(
      bcs_flwheeldirectionvd(bytes, length));
  chassis->mutable_scu_bcs_3_308()->set_bcs_rrwheelspd(
      bcs_rrwheelspd(bytes, length));
  chassis->mutable_scu_bcs_3_308()->set_bcs_rrwheeldirection(
      bcs_rrwheeldirection(bytes, length));
  chassis->mutable_scu_bcs_3_308()->set_bcs_rlwheelspd(
      bcs_rlwheelspd(bytes, length));
  chassis->mutable_scu_bcs_3_308()->set_bcs_rlwheeldirection(
      bcs_rlwheeldirection(bytes, length));
  chassis->mutable_scu_bcs_3_308()->set_bcs_frwheelspd(
      bcs_frwheelspd(bytes, length));
  chassis->mutable_scu_bcs_3_308()->set_bcs_frwheeldirection(
      bcs_frwheeldirection(bytes, length));
  chassis->mutable_scu_bcs_3_308()->set_bcs_flwheelspd(
      bcs_flwheelspd(bytes, length));
  chassis->mutable_scu_bcs_3_308()->set_bcs_flwheeldirection(
      bcs_flwheeldirection(bytes, length));
}

// config detail: {'description': 'Rear right wheel speed valid data', 'enum':
// {0: 'BCS_RRWHEELSPDVD_INVALID', 1: 'BCS_RRWHEELSPDVD_VALID'},
// 'precision': 1.0, 'len': 1, 'name': 'bcs_rrwheelspdvd', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 57, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': '-'}
Scu_bcs_3_308::Bcs_rrwheelspdvdType Scubcs3308::bcs_rrwheelspdvd(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(1, 1);

  Scu_bcs_3_308::Bcs_rrwheelspdvdType ret =
      static_cast<Scu_bcs_3_308::Bcs_rrwheelspdvdType>(x);
  return ret;
}

// config detail: {'description': 'Rear right wheel speed direction valid data',
// 'enum': {0: 'BCS_RRWHEELDIRECTIONVD_INVALID', 1:
// 'BCS_RRWHEELDIRECTIONVD_VALID'}, 'precision': 1.0, 'len': 1, 'name':
// 'bcs_rrwheeldirectionvd', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 58, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': '-'}
Scu_bcs_3_308::Bcs_rrwheeldirectionvdType Scubcs3308::bcs_rrwheeldirectionvd(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(2, 1);

  Scu_bcs_3_308::Bcs_rrwheeldirectionvdType ret =
      static_cast<Scu_bcs_3_308::Bcs_rrwheeldirectionvdType>(x);
  return ret;
}

// config detail: {'description': 'Rear left wheel speed valid data', 'enum':
// {0: 'BCS_RLWHEELSPDVD_INVALID', 1: 'BCS_RLWHEELSPDVD_VALID'},
// 'precision': 1.0, 'len': 1, 'name': 'bcs_rlwheelspdvd', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 41, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': '-'}
Scu_bcs_3_308::Bcs_rlwheelspdvdType Scubcs3308::bcs_rlwheelspdvd(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(1, 1);

  Scu_bcs_3_308::Bcs_rlwheelspdvdType ret =
      static_cast<Scu_bcs_3_308::Bcs_rlwheelspdvdType>(x);
  return ret;
}

// config detail: {'description': 'Rear left wheel speed direction valid data',
// 'enum': {0: 'BCS_RLWHEELDIRECTIONVD_INVALID', 1:
// 'BCS_RLWHEELDIRECTIONVD_VALID'}, 'precision': 1.0, 'len': 1, 'name':
// 'bcs_rlwheeldirectionvd', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 42, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': '-'}
Scu_bcs_3_308::Bcs_rlwheeldirectionvdType Scubcs3308::bcs_rlwheeldirectionvd(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(2, 1);

  Scu_bcs_3_308::Bcs_rlwheeldirectionvdType ret =
      static_cast<Scu_bcs_3_308::Bcs_rlwheeldirectionvdType>(x);
  return ret;
}

// config detail: {'description': 'Front right wheel speed valid data', 'enum':
// {0: 'BCS_FRWHEELSPDVD_INVALID', 1: 'BCS_FRWHEELSPDVD_VALID'},
// 'precision': 1.0, 'len': 1, 'name': 'bcs_frwheelspdvd', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 25, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': '-'}
Scu_bcs_3_308::Bcs_frwheelspdvdType Scubcs3308::bcs_frwheelspdvd(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(1, 1);

  Scu_bcs_3_308::Bcs_frwheelspdvdType ret =
      static_cast<Scu_bcs_3_308::Bcs_frwheelspdvdType>(x);
  return ret;
}

// config detail: {'description': 'Front right wheel speed direction valid
// data', 'enum': {0: 'BCS_FRWHEELDIRECTIONVD_INVALID', 1:
// 'BCS_FRWHEELDIRECTIONVD_VALID'}, 'precision': 1.0, 'len': 1, 'name':
// 'bcs_frwheeldirectionvd', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 26, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': '-'}
Scu_bcs_3_308::Bcs_frwheeldirectionvdType Scubcs3308::bcs_frwheeldirectionvd(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(2, 1);

  Scu_bcs_3_308::Bcs_frwheeldirectionvdType ret =
      static_cast<Scu_bcs_3_308::Bcs_frwheeldirectionvdType>(x);
  return ret;
}

// config detail: {'description': 'Front left wheel speed valid data', 'enum':
// {0: 'BCS_FLWHEELSPDVD_INVALID', 1: 'BCS_FLWHEELSPDVD_VALID'},
// 'precision': 1.0, 'len': 1, 'name': 'bcs_flwheelspdvd', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': '-'}
Scu_bcs_3_308::Bcs_flwheelspdvdType Scubcs3308::bcs_flwheelspdvd(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  Scu_bcs_3_308::Bcs_flwheelspdvdType ret =
      static_cast<Scu_bcs_3_308::Bcs_flwheelspdvdType>(x);
  return ret;
}

// config detail: {'description': 'Front left wheel speed direction valid data',
// 'enum': {0: 'BCS_FLWHEELDIRECTIONVD_INVALID', 1:
// 'BCS_FLWHEELDIRECTIONVD_VALID'}, 'precision': 1.0, 'len': 1, 'name':
// 'bcs_flwheeldirectionvd', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 10, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': '-'}
Scu_bcs_3_308::Bcs_flwheeldirectionvdType Scubcs3308::bcs_flwheeldirectionvd(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  Scu_bcs_3_308::Bcs_flwheeldirectionvdType ret =
      static_cast<Scu_bcs_3_308::Bcs_flwheeldirectionvdType>(x);
  return ret;
}

// config detail: {'description': 'Rear right wheel speed', 'offset': 0.0,
// 'precision': 0.05625, 'len': 13, 'name': 'bcs_rrwheelspd', 'is_signed_var':
// False, 'physical_range': '[0|240]', 'bit': 55, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'km/h'}
double Scubcs3308::bcs_rrwheelspd(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'description': 'Rear right wheel speed direction', 'enum':
// {0: 'BCS_RRWHEELDIRECTION_FORWARD', 1: 'BCS_RRWHEELDIRECTION_BACKWARD'},
// 'precision': 1.0, 'len': 1, 'name': 'bcs_rrwheeldirection', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 56, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': '-'}
Scu_bcs_3_308::Bcs_rrwheeldirectionType Scubcs3308::bcs_rrwheeldirection(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 1);

  Scu_bcs_3_308::Bcs_rrwheeldirectionType ret =
      static_cast<Scu_bcs_3_308::Bcs_rrwheeldirectionType>(x);
  return ret;
}

// config detail: {'description': 'Rear left wheel speed', 'offset': 0.0,
// 'precision': 0.05625, 'len': 13, 'name': 'bcs_rlwheelspd', 'is_signed_var':
// False, 'physical_range': '[0|240]', 'bit': 39, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'km/h'}
double Scubcs3308::bcs_rlwheelspd(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'description': 'Rear left wheel speed direction', 'enum': {0:
// 'BCS_RLWHEELDIRECTION_FORWARD', 1: 'BCS_RLWHEELDIRECTION_BACKWARD'},
// 'precision': 1.0, 'len': 1, 'name': 'bcs_rlwheeldirection', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 40, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': '-'}
Scu_bcs_3_308::Bcs_rlwheeldirectionType Scubcs3308::bcs_rlwheeldirection(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 1);

  Scu_bcs_3_308::Bcs_rlwheeldirectionType ret =
      static_cast<Scu_bcs_3_308::Bcs_rlwheeldirectionType>(x);
  return ret;
}

// config detail: {'description': 'Front right wheel speed', 'offset': 0.0,
// 'precision': 0.05625, 'len': 13, 'name': 'bcs_frwheelspd', 'is_signed_var':
// False, 'physical_range': '[0|240]', 'bit': 23, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'km/h'}
double Scubcs3308::bcs_frwheelspd(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'description': 'Front right wheel speed direction', 'enum':
// {0: 'BCS_FRWHEELDIRECTION_FORWARD', 1: 'BCS_FRWHEELDIRECTION_BACKWARD'},
// 'precision': 1.0, 'len': 1, 'name': 'bcs_frwheeldirection', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 24, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': '-'}
Scu_bcs_3_308::Bcs_frwheeldirectionType Scubcs3308::bcs_frwheeldirection(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 1);

  Scu_bcs_3_308::Bcs_frwheeldirectionType ret =
      static_cast<Scu_bcs_3_308::Bcs_frwheeldirectionType>(x);
  return ret;
}

// config detail: {'description': 'Front left wheel speed', 'offset': 0.0,
// 'precision': 0.05625, 'len': 13, 'name': 'bcs_flwheelspd', 'is_signed_var':
// False, 'physical_range': '[0|240]', 'bit': 7, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'km/h'}
double Scubcs3308::bcs_flwheelspd(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'description': 'Front left wheel speed direction', 'enum':
// {0: 'BCS_FLWHEELDIRECTION_FORWARD', 1: 'BCS_FLWHEELDIRECTION_BACKWARD'},
// 'precision': 1.0, 'len': 1, 'name': 'bcs_flwheeldirection', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': '-'}
Scu_bcs_3_308::Bcs_flwheeldirectionType Scubcs3308::bcs_flwheeldirection(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  Scu_bcs_3_308::Bcs_flwheeldirectionType ret =
      static_cast<Scu_bcs_3_308::Bcs_flwheeldirectionType>(x);
  return ret;
}
}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
