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

#include "modules/canbus_vehicle/devkit/protocol/vin_resp1_514.h"

#include <string>

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::drivers::canbus::Byte;

Vinresp1514::Vinresp1514() {}
const int32_t Vinresp1514::ID = 0x514;

void Vinresp1514::Parse(const std::uint8_t* bytes, int32_t length,
                        Devkit* chassis) const {
  chassis->mutable_vin_resp1_514()->set_vin07(
      vin07(bytes, length));
  chassis->mutable_vin_resp1_514()->set_vin06(
      vin06(bytes, length));
  chassis->mutable_vin_resp1_514()->set_vin05(
      vin05(bytes, length));
  chassis->mutable_vin_resp1_514()->set_vin04(
      vin04(bytes, length));
  chassis->mutable_vin_resp1_514()->set_vin03(
      vin03(bytes, length));
  chassis->mutable_vin_resp1_514()->set_vin02(
      vin02(bytes, length));
  chassis->mutable_vin_resp1_514()->set_vin01(
      vin01(bytes, length));
  chassis->mutable_vin_resp1_514()->set_vin00(
      vin00(bytes, length));
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'vin07',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
std::string Vinresp1514::vin07(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}

// config detail: {'bit': 55, 'is_signed_var': False, 'len': 8, 'name': 'vin06',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
std::string Vinresp1514::vin06(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}

// config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name': 'vin05',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
std::string Vinresp1514::vin05(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': False, 'len': 8, 'name': 'vin04',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
std::string Vinresp1514::vin04(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}

// config detail: {'bit': 31, 'is_signed_var': False, 'len': 8, 'name': 'vin03',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
std::string Vinresp1514::vin03(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': False, 'len': 8, 'name': 'vin02',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
std::string Vinresp1514::vin02(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name': 'vin01',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
std::string Vinresp1514::vin01(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'vin00',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
std::string Vinresp1514::vin00(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}
}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
