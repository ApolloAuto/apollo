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

#include "modules/canbus_vehicle/neolix_edu/protocol/aeb_frontwheelspeed_353.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

Aebfrontwheelspeed353::Aebfrontwheelspeed353() {}
const int32_t Aebfrontwheelspeed353::ID = 0x353;

void Aebfrontwheelspeed353::Parse(const std::uint8_t* bytes, int32_t length,
                                  Neolix_edu* chassis) const {
  chassis->mutable_aeb_frontwheelspeed_353()->set_vehiclespeedvalid(
      vehiclespeedvalid(bytes, length));
  chassis->mutable_aeb_frontwheelspeed_353()->set_vehiclespeed(
      vehiclespeed(bytes, length));
  chassis->mutable_aeb_frontwheelspeed_353()->set_vehiclerealdirect(
      vehiclerealdirect(bytes, length));
  chassis->mutable_aeb_frontwheelspeed_353()->set_wheelspeed_fl_valid(
      wheelspeed_fl_valid(bytes, length));
  chassis->mutable_aeb_frontwheelspeed_353()->set_wheelspeed_fl(
      wheelspeed_fl(bytes, length));
  chassis->mutable_aeb_frontwheelspeed_353()->set_wheelspeed_fr_valid(
      wheelspeed_fr_valid(bytes, length));
  chassis->mutable_aeb_frontwheelspeed_353()->set_wheelspeed_fr(
      wheelspeed_fr(bytes, length));
  chassis->mutable_aeb_frontwheelspeed_353()->set_wheelspeed_fl_direct(
      wheelspeed_fl_direct(bytes, length));
  chassis->mutable_aeb_frontwheelspeed_353()->set_wheelspeed_fr_direct(
      wheelspeed_fr_direct(bytes, length));
  chassis->mutable_aeb_frontwheelspeed_353()->set_alivecounter_front(
      alivecounter_front(bytes, length));
  chassis->mutable_aeb_frontwheelspeed_353()->set_checksum_front(
      checksum_front(bytes, length));
}

// config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'vehiclespeedvalid', 'is_signed_var':
// False, 'physical_range': '[0.0|1.0]', 'bit': 7, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': 'bit'}
bool Aebfrontwheelspeed353::vehiclespeedvalid(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'vehiclespeed', 'offset': 0.0, 'precision': 0.05625,
// 'len': 13, 'is_signed_var': False, 'physical_range': '[0.0|460.69]', 'bit':
// 4, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Km/h'}
double Aebfrontwheelspeed353::vehiclespeed(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 5);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'description': '0x0:Invalid;0x1:D;0x2:N;0x3:R', 'offset':
// 0.0, 'precision': 1.0, 'len': 2, 'name': 'vehiclerealdirect',
// 'is_signed_var': False, 'physical_range': '[0.0|3.0]', 'bit': 6, 'type':
// 'double', 'order': 'motorola', 'physical_unit': 'bit'}
double Aebfrontwheelspeed353::vehiclerealdirect(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 2);

  double ret = x;
  return ret;
}

// config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'wheelspeed_fl_valid', 'is_signed_var':
// False, 'physical_range': '[0.0|1.0]', 'bit': 23, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': 'bit'}
bool Aebfrontwheelspeed353::wheelspeed_fl_valid(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'wheelspeed_fl', 'offset': 0.0, 'precision': 0.01,
// 'len': 15, 'is_signed_var': False, 'physical_range': '[0.0|327.67]', 'bit':
// 22, 'type': 'double', 'order': 'motorola', 'physical_unit': 'km/h'}
double Aebfrontwheelspeed353::wheelspeed_fl(const std::uint8_t* bytes,
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
// 'precision': 1.0, 'len': 1, 'name': 'wheelspeed_fr_valid', 'is_signed_var':
// False, 'physical_range': '[0.0|1.0]', 'bit': 39, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': 'bit'}
bool Aebfrontwheelspeed353::wheelspeed_fr_valid(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'wheelspeed_fr', 'offset': 0.0, 'precision': 0.01,
// 'len': 15, 'is_signed_var': False, 'physical_range': '[0.0|327.67]', 'bit':
// 38, 'type': 'double', 'order': 'motorola', 'physical_unit': 'km/h'}
double Aebfrontwheelspeed353::wheelspeed_fr(const std::uint8_t* bytes,
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
// 0.0, 'precision': 1.0, 'len': 2, 'name': 'wheelspeed_fl_direct',
// 'is_signed_var': False, 'physical_range': '[0.0|3.0]', 'bit': 53, 'type':
// 'double', 'order': 'motorola', 'physical_unit': 'bit'}
double Aebfrontwheelspeed353::wheelspeed_fl_direct(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 2);

  double ret = x;
  return ret;
}

// config detail: {'description': '0x0:Invalid;0x1:D;0x2:N;0x3:R', 'offset':
// 0.0, 'precision': 1.0, 'len': 2, 'name': 'wheelspeed_fr_direct',
// 'is_signed_var': False, 'physical_range': '[0.0|3.0]', 'bit': 55, 'type':
// 'double', 'order': 'motorola', 'physical_unit': 'bit'}
double Aebfrontwheelspeed353::wheelspeed_fr_direct(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 2);

  double ret = x;
  return ret;
}

// config detail: {'name': 'alivecounter_front', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0.0|15.0]', 'bit': 51, 'type': 'double', 'order': 'motorola',
// 'physical_unit': ''}
double Aebfrontwheelspeed353::alivecounter_front(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  double ret = x;
  return ret;
}

// config detail: {'name': 'checksum_front', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0.0|255.0]', 'bit': 63,
// 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Aebfrontwheelspeed353::checksum_front(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  double ret = x;
  return ret;
}
}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
