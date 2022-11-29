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

#include "modules/canbus_vehicle/neolix_edu/protocol/aeb_systemstate_11.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

Aebsystemstate11::Aebsystemstate11() {}
const int32_t Aebsystemstate11::ID = 0x11;

void Aebsystemstate11::Parse(const std::uint8_t* bytes, int32_t length,
                             Neolix_edu* chassis) const {
  chassis->mutable_aeb_systemstate_11()->set_aeb_state(
      aeb_state(bytes, length));
  chassis->mutable_aeb_systemstate_11()->set_aeb_brakestate(
      aeb_brakestate(bytes, length));
  chassis->mutable_aeb_systemstate_11()->set_faultrank(
      faultrank(bytes, length));
  chassis->mutable_aeb_systemstate_11()->set_currenttemperature(
      currenttemperature(bytes, length));
  chassis->mutable_aeb_systemstate_11()->set_pas_f1_stop(
      pas_f1_stop(bytes, length));
  chassis->mutable_aeb_systemstate_11()->set_pas_f2_stop(
      pas_f2_stop(bytes, length));
  chassis->mutable_aeb_systemstate_11()->set_pas_f3_stop(
      pas_f3_stop(bytes, length));
  chassis->mutable_aeb_systemstate_11()->set_pas_f4_stop(
      pas_f4_stop(bytes, length));
  chassis->mutable_aeb_systemstate_11()->set_pas_b1_stop(
      pas_b1_stop(bytes, length));
  chassis->mutable_aeb_systemstate_11()->set_pas_b2_stop(
      pas_b2_stop(bytes, length));
  chassis->mutable_aeb_systemstate_11()->set_pas_b3_stop(
      pas_b3_stop(bytes, length));
  chassis->mutable_aeb_systemstate_11()->set_pas_b4_stop(
      pas_b4_stop(bytes, length));
  chassis->mutable_aeb_systemstate_11()->set_aeb_livecounter_rear(
    aeb_livecounter_rear(bytes, length));
  chassis->mutable_aeb_systemstate_11()->set_aeb_cheksum(
      aeb_cheksum(bytes, length));
}

// config detail: {'description': '0x00:read only;0x01:brake enable', 'offset':
// 0.0, 'precision': 1.0, 'len': 2, 'name': 'aeb_state', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 1, 'type': 'int', 'order': 'motorola',
// 'physical_unit': ''}
int Aebsystemstate11::aeb_state(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'description': '0x00:off;0x01:on', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'aeb_brakestate', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 2, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': ''}
bool Aebsystemstate11::aeb_brakestate(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Nomal;0x1:Level 1;0x2:Level 2;0x3:Level
// 3;0x4:Level 4;0x5:Level 5;0x6:Reserved;0x7:Reserved', 'offset': 0.0,
// 'precision': 1.0, 'len': 3, 'name': 'faultrank', 'is_signed_var': False,
// 'physical_range': '[0|5]', 'bit': 10, 'type': 'int', 'order': 'motorola',
// 'physical_unit': ''}
int Aebsystemstate11::faultrank(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 3);

  int ret = x;
  return ret;
}

// config detail: {'name': 'currenttemperature', 'offset': -40.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|120]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit':
// ''}
int Aebsystemstate11::currenttemperature(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + -40.000000;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'pas_f1_stop', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 24, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': ''}
bool Aebsystemstate11::pas_f1_stop(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'pas_f2_stop', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 25, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': ''}
bool Aebsystemstate11::pas_f2_stop(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'pas_f3_stop', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 26, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': ''}
bool Aebsystemstate11::pas_f3_stop(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'pas_f4_stop', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 27, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': ''}
bool Aebsystemstate11::pas_f4_stop(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'pas_b1_stop', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 28, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': ''}
bool Aebsystemstate11::pas_b1_stop(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'pas_b2_stop', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 29, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': ''}
bool Aebsystemstate11::pas_b2_stop(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'pas_b3_stop', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 30, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': ''}
bool Aebsystemstate11::pas_b3_stop(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'pas_b4_stop', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 31, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': ''}
bool Aebsystemstate11::pas_b4_stop(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'aeb_livecounter_rear', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Aebsystemstate11::aeb_livecounter_rear(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'aeb_cheksum', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
// 'type': 'int', 'order': 'motorola', 'physical_unit': 'bit'}
int Aebsystemstate11::aeb_cheksum(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
