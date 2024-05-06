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

#include "modules/canbus_vehicle/devkit/protocol/brake_report_501.h"

#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::drivers::canbus::Byte;

Brakereport501::Brakereport501() {}
const int32_t Brakereport501::ID = 0x501;

void Brakereport501::Parse(const std::uint8_t* bytes, int32_t length,
                           Devkit* chassis) const {
  chassis->mutable_brake_report_501()->set_brake_pedal_actual(
      brake_pedal_actual(bytes, length));
  chassis->mutable_brake_report_501()->set_brake_flt2(
      brake_flt2(bytes, length));
  chassis->mutable_brake_report_501()->set_brake_flt1(
      brake_flt1(bytes, length));
  chassis->mutable_brake_report_501()->set_brake_en_state(
      brake_en_state(bytes, length));
}

// config detail: {'name': 'brake_pedal_actual', 'offset': 0.0, 'precision':
// 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit':
// 31, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
double Brakereport501::brake_pedal_actual(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'description': 'Brake system communication fault', 'enum':
// {0: 'BRAKE_FLT2_NO_FAULT', 1: 'BRAKE_FLT2_BRAKE_SYSTEM_COMUNICATION_FAULT'},
// 'precision': 1.0, 'len': 8, 'name': 'brake_flt2', 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Brake_report_501::Brake_flt2Type Brakereport501::brake_flt2(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Brake_report_501::Brake_flt2Type ret =
      static_cast<Brake_report_501::Brake_flt2Type>(x);
  return ret;
}

// config detail: {'description': 'Brake system hardware fault', 'enum': {0:
// 'BRAKE_FLT1_NO_FAULT', 1: 'BRAKE_FLT1_BRAKE_SYSTEM_HARDWARE_FAULT'},
// 'precision': 1.0, 'len': 8, 'name': 'brake_flt1', 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Brake_report_501::Brake_flt1Type Brakereport501::brake_flt1(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Brake_report_501::Brake_flt1Type ret =
      static_cast<Brake_report_501::Brake_flt1Type>(x);
  return ret;
}

// config detail: {'name': 'brake_en_state', 'enum': {0:
// 'BRAKE_EN_STATE_MANUAL', 1: 'BRAKE_EN_STATE_AUTO', 2:
// 'BRAKE_EN_STATE_TAKEOVER', 3: 'BRAKE_EN_STATE_STANDBY'}, 'precision': 1.0,
// 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]',
// 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Brake_report_501::Brake_en_stateType Brakereport501::brake_en_state(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  Brake_report_501::Brake_en_stateType ret =
      static_cast<Brake_report_501::Brake_en_stateType>(x);
  return ret;
}
}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
