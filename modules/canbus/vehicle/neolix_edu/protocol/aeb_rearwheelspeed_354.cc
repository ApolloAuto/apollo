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

#include "modules/canbus/vehicle/neolix_edu/protocol/aeb_rearwheelspeed_354.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

Aebrearwheelspeed354::Aebrearwheelspeed354() {}
const int32_t Aebrearwheelspeed354::ID = 0x354;

void Aebrearwheelspeed354::Parse(const std::uint8_t* bytes, int32_t length,
                                 ChassisDetail* chassis) const {
  chassis->mutable_neolix_edu()
      ->mutable_aeb_rearwheelspeed_354()
      ->set_wheelspeed_rl_valid(wheelspeed_rl_valid(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_aeb_rearwheelspeed_354()
      ->set_wheelspeed_rl(wheelspeed_rl(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_aeb_rearwheelspeed_354()
      ->set_wheelspeed_rr_valid(wheelspeed_rr_valid(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_aeb_rearwheelspeed_354()
      ->set_wheelspeed_rr(wheelspeed_rr(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_aeb_rearwheelspeed_354()
      ->set_wheelspeed_rl_direct(wheelspeed_rl_direct(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_aeb_rearwheelspeed_354()
      ->set_wheelspeed_rr_direct(wheelspeed_rr_direct(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_aeb_rearwheelspeed_354()
      ->set_alivecounter_rear(alivecounter_rear(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_aeb_rearwheelspeed_354()
      ->set_checksum_rear(checksum_rear(bytes, length));
}

// config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'wheelspeed_rl_valid', 'is_signed_var':
// False, 'physical_range': '[0.0|1.0]', 'bit': 23, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': 'bit'}
bool Aebrearwheelspeed354::wheelspeed_rl_valid(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'wheelspeed_rl', 'offset': 0.0, 'precision': 0.01,
// 'len': 15, 'is_signed_var': False, 'physical_range': '[0.0|327.67]', 'bit':
// 22, 'type': 'double', 'order': 'motorola', 'physical_unit': 'km/h'}
double Aebrearwheelspeed354::wheelspeed_rl(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 7);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'wheelspeed_rr_valid', 'is_signed_var':
// False, 'physical_range': '[0.0|1.0]', 'bit': 39, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': 'bit'}
bool Aebrearwheelspeed354::wheelspeed_rr_valid(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'wheelspeed_rr', 'offset': 0.0, 'precision': 0.01,
// 'len': 15, 'is_signed_var': False, 'physical_range': '[0.0|327.67]', 'bit':
// 38, 'type': 'double', 'order': 'motorola', 'physical_unit': 'km/h'}
double Aebrearwheelspeed354::wheelspeed_rr(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 7);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'description': '0x0:Invalid;0x1:D;0x2:N;0x3:R', 'offset':
// 0.0, 'precision': 1.0, 'len': 2, 'name': 'wheelspeed_rl_direct',
// 'is_signed_var': False, 'physical_range': '[0.0|3.0]', 'bit': 53, 'type':
// 'double', 'order': 'motorola', 'physical_unit': 'bit'}
double Aebrearwheelspeed354::wheelspeed_rl_direct(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 2);

  double ret = x;
  return ret;
}

// config detail: {'description': '0x0:Invalid;0x1:D;0x2:N;0x3:R', 'offset':
// 0.0, 'precision': 1.0, 'len': 2, 'name': 'wheelspeed_rr_direct',
// 'is_signed_var': False, 'physical_range': '[0.0|3.0]', 'bit': 55, 'type':
// 'double', 'order': 'motorola', 'physical_unit': 'bit'}
double Aebrearwheelspeed354::wheelspeed_rr_direct(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 2);

  double ret = x;
  return ret;
}

// config detail: {'name': 'alivecounter_rear', 'offset': 0.0, 'precision': 1.0,
// 'len': 4, 'is_signed_var': False, 'physical_range': '[0.0|15.0]', 'bit': 51,
// 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Aebrearwheelspeed354::alivecounter_rear(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  double ret = x;
  return ret;
}

// config detail: {'name': 'checksum_rear', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0.0|255.0]', 'bit': 63,
// 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Aebrearwheelspeed354::checksum_rear(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  double ret = x;
  return ret;
}
}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
