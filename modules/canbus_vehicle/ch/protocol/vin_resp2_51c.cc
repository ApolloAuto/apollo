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

#include "modules/canbus_vehicle/ch/protocol/vin_resp2_51c.h"

#include <string>

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Vinresp251c::Vinresp251c() {}
const int32_t Vinresp251c::ID = 0x51C;

void Vinresp251c::Parse(const std::uint8_t* bytes, int32_t length,
                        Ch* chassis) const {
  chassis->mutable_vin_resp2_51c()->set_vin16(
      vin16(bytes, length));
  chassis->mutable_vin_resp2_51c()->set_vin15(
      vin15(bytes, length));
  chassis->mutable_vin_resp2_51c()->set_vin14(
      vin14(bytes, length));
  chassis->mutable_vin_resp2_51c()->set_vin13(
      vin13(bytes, length));
  chassis->mutable_vin_resp2_51c()->set_vin12(
      vin12(bytes, length));
  chassis->mutable_vin_resp2_51c()->set_vin11(
      vin11(bytes, length));
  chassis->mutable_vin_resp2_51c()->set_vin10(
      vin10(bytes, length));
  chassis->mutable_vin_resp2_51c()->set_vin09(
      vin09(bytes, length));
}

// config detail: {'bit': 56, 'description': 'VIN Response', 'is_signed_var':
// False, 'len': 8, 'name': 'vin16', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
std::string Vinresp251c::vin16(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}

// config detail: {'bit': 48, 'description': 'VIN Response', 'is_signed_var':
// False, 'len': 8, 'name': 'vin15', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
std::string Vinresp251c::vin15(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}

// config detail: {'bit': 40, 'description': 'VIN Response', 'is_signed_var':
// False, 'len': 8, 'name': 'vin14', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
std::string Vinresp251c::vin14(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}

// config detail: {'bit': 32, 'description': 'VIN Response', 'is_signed_var':
// False, 'len': 8, 'name': 'vin13', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
std::string Vinresp251c::vin13(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}

// config detail: {'bit': 24, 'description': 'VIN Response', 'is_signed_var':
// False, 'len': 8, 'name': 'vin12', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
std::string Vinresp251c::vin12(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}

// config detail: {'bit': 16, 'description': 'VIN Response', 'is_signed_var':
// False, 'len': 8, 'name': 'vin11', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
std::string Vinresp251c::vin11(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}

// config detail: {'bit': 8, 'description': 'VIN Response', 'is_signed_var':
// False, 'len': 8, 'name': 'vin10', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
std::string Vinresp251c::vin10(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}

// config detail: {'bit': 0, 'description': 'VIN Response', 'is_signed_var':
// False, 'len': 8, 'name': 'vin09', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
std::string Vinresp251c::vin09(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
