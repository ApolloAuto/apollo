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

#include "modules/canbus/vehicle/wey/protocol/vin_resp1_391.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace wey {

using ::apollo::drivers::canbus::Byte;

Vinresp1391::Vinresp1391() {}
const int32_t Vinresp1391::ID = 0x391;

void Vinresp1391::Parse(const std::uint8_t* bytes, int32_t length,
                        ChassisDetail* chassis) const {
  chassis->mutable_wey()->mutable_vin_resp1_391()->set_vin07(
      vin07(bytes, length));
  chassis->mutable_wey()->mutable_vin_resp1_391()->set_vin06(
      vin06(bytes, length));
  chassis->mutable_wey()->mutable_vin_resp1_391()->set_vin05(
      vin05(bytes, length));
  chassis->mutable_wey()->mutable_vin_resp1_391()->set_vin04(
      vin04(bytes, length));
  chassis->mutable_wey()->mutable_vin_resp1_391()->set_vin03(
      vin03(bytes, length));
  chassis->mutable_wey()->mutable_vin_resp1_391()->set_vin02(
      vin02(bytes, length));
  chassis->mutable_wey()->mutable_vin_resp1_391()->set_vin00(
      vin00(bytes, length));
  chassis->mutable_wey()->mutable_vin_resp1_391()->set_vin01(
      vin01(bytes, length));
}

// config detail: {'name': 'vin07', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 7, 'type':'int',
// 'order': 'motorola', 'physical_unit': ''}
int Vinresp1391::vin07(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vin06', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 15,'type':'int',
// 'order': 'motorola', 'physical_unit': ''}
int Vinresp1391::vin06(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vin05', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 23,'type':'int',
// 'order': 'motorola', 'physical_unit': ''}
int Vinresp1391::vin05(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vin04', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 31,'type':'int',
// 'order': 'motorola', 'physical_unit': ''}
int Vinresp1391::vin04(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vin03', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 39,'type':'int',
// 'order': 'motorola', 'physical_unit': ''}
int Vinresp1391::vin03(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vin02', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 47,'type':'int',
// 'order': 'motorola', 'physical_unit': ''}
int Vinresp1391::vin02(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vin00', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,'type':'int',
// 'order': 'motorola', 'physical_unit': ''}
int Vinresp1391::vin00(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vin01', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 55,'type':'int',
// 'order': 'motorola', 'physical_unit': ''}
int Vinresp1391::vin01(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace wey
}  // namespace canbus
}  // namespace apollo
