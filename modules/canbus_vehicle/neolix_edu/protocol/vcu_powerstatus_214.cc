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

#include "modules/canbus_vehicle/neolix_edu/protocol/vcu_powerstatus_214.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

Vcupowerstatus214::Vcupowerstatus214() {}
const int32_t Vcupowerstatus214::ID = 0x214;

void Vcupowerstatus214::Parse(const std::uint8_t* bytes, int32_t length,
                              Neolix_edu* chassis) const {
  chassis->mutable_vcu_powerstatus_214()->set_vcu_powermode(
      vcu_powermode(bytes, length));
  chassis->mutable_vcu_powerstatus_214()->set_vcu_powermodevalid(
      vcu_powermodevalid(bytes, length));
  chassis->mutable_vcu_powerstatus_214()->set_replacebatterystateindication(
      replacebatterystateindication(bytes, length));
  chassis->mutable_vcu_powerstatus_214()->set_forbidden_aeb_signal(
      forbidden_aeb_signal(bytes, length));
  chassis->mutable_vcu_powerstatus_214()->set_bcu_chargedischargecurrent(
      bcu_chargedischargecurrent(bytes, length));
  chassis->mutable_vcu_powerstatus_214()->set_bcu_batt_internalvoltage(
      bcu_batt_internalvoltage(bytes, length));
  chassis->mutable_vcu_powerstatus_214()->set_vcu_driverinfo_alivecounter(
      vcu_driverinfo_alivecounter(bytes, length));
  chassis->mutable_vcu_powerstatus_214()->set_vcu_driverinfo_checksum(
      vcu_driverinfo_checksum(bytes, length));
}

// config detail: {'description':
// '0x0:OFF;0x1:IG_ON;0x2:Power_ON;0x3:Auto_ON;0x4:Reserved;0x5:Reserved;0x6:Reserved;0x7:Reserved',
// 'offset': 0.0, 'precision': 1.0, 'len': 3, 'name': 'vcu_powermode',
// 'is_signed_var': False, 'physical_range': '[0|7]', 'bit': 11, 'type': 'int',
// 'order': 'motorola', 'physical_unit': 'bit'}
int Vcupowerstatus214::vcu_powermode(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 3);

  int ret = x;
  return ret;
}

// config detail: {'description': '0x0:Not
// Available;0x1:Invalid;0x2:Valid;0x3:Reserved', 'offset': 0.0,
// 'precision': 1.0, 'len': 2, 'name': 'vcu_powermodevalid', 'is_signed_var':
// False, 'physical_range': '[0|3]', 'bit': 13, 'type': 'int', 'order':
// 'motorola', 'physical_unit': 'bit'}
int Vcupowerstatus214::vcu_powermodevalid(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 2);

  int ret = x;
  return ret;
}

// config detail: {'description': '0x0:NotActivate;0x1:Activate', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'replacebatterystateindication',
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 14, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': 'bit'}
bool Vcupowerstatus214::replacebatterystateindication(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

bool Vcupowerstatus214::forbidden_aeb_signal(const std::uint8_t* bytes,
                                             const int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

float Vcupowerstatus214::bcu_chargedischargecurrent(
    const std::uint8_t* bytes, const int32_t length) const {
  Byte t0(bytes + 2);
  Byte t1(bytes + 3);
  int32_t x1 = t0.get_byte(0, 8);
  int32_t x2 = t1.get_byte(0, 8);

  int ret = (x1 << 8 | x2) * 0.02 - 400;
  return ret;
}

float Vcupowerstatus214::bcu_batt_internalvoltage(const std::uint8_t* bytes,
                                                  const int32_t length) const {
  Byte t0(bytes + 4);
  Byte t1(bytes + 5);
  int32_t x1 = t0.get_byte(0, 8);
  int32_t x2 = t1.get_byte(0, 8);

  int ret = (x1 << 8 | x2) * 0.01;
  return ret;
}

// config detail: {'name': 'vcu_driverinfo_alivecounter', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0|15]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit':
// 'bit'}
int Vcupowerstatus214::vcu_driverinfo_alivecounter(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_driverinfo_checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
// 'bit'}
int Vcupowerstatus214::vcu_driverinfo_checksum(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
