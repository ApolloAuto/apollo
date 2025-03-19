/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/neolix_edu/protocol/aeb_wheelimpulse_355.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

Aebwheelimpulse355::Aebwheelimpulse355() {}
const int32_t Aebwheelimpulse355::ID = 0x355;

void Aebwheelimpulse355::Parse(const std::uint8_t* bytes, int32_t length,
                               ChassisDetail* chassis) const {
  chassis->mutable_neolix_edu()->mutable_aeb_wheelimpulse_355()->set_flimpulse(
      flimpulse(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_aeb_wheelimpulse_355()
      ->set_flimpulsevalid(flimpulsevalid(bytes, length));
  chassis->mutable_neolix_edu()->mutable_aeb_wheelimpulse_355()->set_frimpulse(
      frimpulse(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_aeb_wheelimpulse_355()
      ->set_frimpulsevalid(frimpulsevalid(bytes, length));
  chassis->mutable_neolix_edu()->mutable_aeb_wheelimpulse_355()->set_rlimpulse(
      rlimpulse(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_aeb_wheelimpulse_355()
      ->set_rlimpulsevalid(rlimpulsevalid(bytes, length));
  chassis->mutable_neolix_edu()->mutable_aeb_wheelimpulse_355()->set_rrimpulse(
      rrimpulse(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_aeb_wheelimpulse_355()
      ->set_rrimpulsevalid(rrimpulsevalid(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_aeb_wheelimpulse_355()
      ->set_alivecounter(alivecounter(bytes, length));
  chassis->mutable_neolix_edu()->mutable_aeb_wheelimpulse_355()->set_checksum(
      checksum(bytes, length));
}

// config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 10, 'name': 'flimpulse', 'is_signed_var': False,
// 'physical_range': '[0.0|1023.0]', 'bit': 7, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'bit'}
double Aebwheelimpulse355::flimpulse(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(6, 2);
  x <<= 2;
  x |= t;

  double ret = x;
  return ret;
}

// config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'flimpulsevalid', 'is_signed_var': False,
// 'physical_range': '[0.0|1.0]', 'bit': 13, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': 'bit'}
bool Aebwheelimpulse355::flimpulsevalid(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'frimpulse', 'offset': 0.0, 'precision': 1.0, 'len':
// 10, 'is_signed_var': False, 'physical_range': '[0.0|1023.0]', 'bit': 12,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'km/h'}
double Aebwheelimpulse355::frimpulse(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 5);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret = x;
  return ret;
}

// config detail: {'name': 'frimpulsevalid', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0.0|1.0]', 'bit': 18,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': 'km/h'}
bool Aebwheelimpulse355::frimpulsevalid(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 10, 'name': 'rlimpulse', 'is_signed_var': False,
// 'physical_range': '[0.0|1023.0]', 'bit': 17, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'bit'}
double Aebwheelimpulse355::rlimpulse(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 2);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x;
  return ret;
}

// config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'rlimpulsevalid', 'is_signed_var': False,
// 'physical_range': '[0.0|1.0]', 'bit': 39, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': 'bit'}
bool Aebwheelimpulse355::rlimpulsevalid(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rrimpulse', 'offset': 0.0, 'precision': 1.0, 'len':
// 10, 'is_signed_var': False, 'physical_range': '[0.0|1023.0]', 'bit': 38,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'km/h'}
double Aebwheelimpulse355::rrimpulse(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 7);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(5, 3);
  x <<= 3;
  x |= t;

  double ret = x;
  return ret;
}

// config detail: {'name': 'rrimpulsevalid', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0.0|1.0]', 'bit': 44,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': 'km/h'}
bool Aebwheelimpulse355::rrimpulsevalid(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'alivecounter', 'offset': 0.0, 'precision': 1.0,
// 'len': 4, 'is_signed_var': False, 'physical_range': '[0.0|15.0]', 'bit': 51,
// 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Aebwheelimpulse355::alivecounter(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  double ret = x;
  return ret;
}

// config detail: {'name': 'checksum', 'offset': 0.0, 'precision': 1.0, 'len':
// 8, 'is_signed_var': False, 'physical_range': '[0.0|255.0]', 'bit': 63,
// 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Aebwheelimpulse355::checksum(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  double ret = x;
  return ret;
}
}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
