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

#include "modules/canbus_vehicle/neolix_edu/protocol/pas_1st_data_311.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

Pas1stdata311::Pas1stdata311() {}
const int32_t Pas1stdata311::ID = 0x311;

void Pas1stdata311::Parse(const std::uint8_t* bytes, int32_t length,
                          Neolix_edu* chassis) const {
  chassis->mutable_pas_1st_data_311()->set_pasdistance4(
      pasdistance4(bytes, length));
  chassis->mutable_pas_1st_data_311()->set_pasdistance3(
      pasdistance3(bytes, length));
  chassis->mutable_pas_1st_data_311()->set_pas_f1_status(
      pas_f1_status(bytes, length));
  chassis->mutable_pas_1st_data_311()->set_pas_f2_status(
      pas_f2_status(bytes, length));
  chassis->mutable_pas_1st_data_311()->set_pas_f3_status(
      pas_f3_status(bytes, length));
  chassis->mutable_pas_1st_data_311()->set_pas_f4_status(
      pas_f4_status(bytes, length));
  chassis->mutable_pas_1st_data_311()->set_pasdistance2(
      pasdistance2(bytes, length));
  chassis->mutable_pas_1st_data_311()->set_pasdistance1(
      pasdistance1(bytes, length));
}

// config detail: {'description': 'phy=int*2;0xFF:no obstacle', 'offset': 0.0,
// 'precision': 2.0, 'len': 8, 'name': 'pasdistance4', 'is_signed_var': False,
// 'physical_range': '[0|510]', 'bit': 39, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'cm'}
double Pas1stdata311::pasdistance4(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'description': 'phy=int*2;0xFF:no obstacle', 'offset': 0.0,
// 'precision': 2.0, 'len': 8, 'name': 'pasdistance3', 'is_signed_var': False,
// 'physical_range': '[0|510]', 'bit': 31, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'cm'}
double Pas1stdata311::pasdistance3(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'pas_f1_status', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 0, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': 'bit'}
bool Pas1stdata311::pas_f1_status(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'pas_f2_status', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': 'bit'}
bool Pas1stdata311::pas_f2_status(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'pas_f3_status', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 2, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': 'bit'}
bool Pas1stdata311::pas_f3_status(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'pas_f4_status', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 3, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': 'bit'}
bool Pas1stdata311::pas_f4_status(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'phy=int*2;0xFF:no obstacle', 'offset': 0.0,
// 'precision': 2.0, 'len': 8, 'name': 'pasdistance2', 'is_signed_var': False,
// 'physical_range': '[0|510]', 'bit': 23, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'cm'}
double Pas1stdata311::pasdistance2(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'name': 'pasdistance1', 'offset': 0.0, 'precision': 2.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|510]', 'bit': 15,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
double Pas1stdata311::pasdistance1(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}
}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
