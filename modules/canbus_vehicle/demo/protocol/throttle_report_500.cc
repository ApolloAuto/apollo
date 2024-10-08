/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/demo/protocol/throttle_report_500.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace demo {

using ::apollo::drivers::canbus::Byte;

Throttlereport500::Throttlereport500() {}
const int32_t Throttlereport500::ID = 0x500;

void Throttlereport500::Parse(const std::uint8_t* bytes, int32_t length,
                              Demo* chassis) const {
  chassis->mutable_throttle_report_500()->set_throttle_pedal_actual(
      throttle_pedal_actual(bytes, length));
  chassis->mutable_throttle_report_500()->set_throttle_flt2(
      throttle_flt2(bytes, length));
  chassis->mutable_throttle_report_500()->set_throttle_flt1(
      throttle_flt1(bytes, length));
  chassis->mutable_throttle_report_500()->set_throttle_en_state(
      throttle_en_state(bytes, length));
}

// config detail: {'bit': 31, 'description': 'command', 'is_signed_var': False,
// 'len': 16, 'name': 'throttle_pedal_actual', 'offset': 0.0, 'order':
// 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision':
// 0.1, 'signal_type': 'command', 'type': 'double'}
double Throttlereport500::throttle_pedal_actual(const std::uint8_t* bytes,
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

// config detail: {'bit': 23, 'description': 'Drive system communication fault',
// 'enum': {0: 'THROTTLE_FLT2_NO_FAULT', 1:
// 'THROTTLE_FLT2_DRIVE_SYSTEM_COMUNICATION_FAULT'}, 'is_signed_var': False,
// 'len': 8, 'name': 'throttle_flt2', 'offset': 0.0, 'order': 'motorola',
// 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Throttle_report_500::Throttle_flt2Type Throttlereport500::throttle_flt2(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Throttle_report_500::Throttle_flt2Type ret =
      static_cast<Throttle_report_500::Throttle_flt2Type>(x);
  return ret;
}

// config detail: {'bit': 15, 'description': 'Drive system hardware fault',
// 'enum': {0: 'THROTTLE_FLT1_NO_FAULT', 1:
// 'THROTTLE_FLT1_DRIVE_SYSTEM_HARDWARE_FAULT'}, 'is_signed_var': False, 'len':
// 8, 'name': 'throttle_flt1', 'offset': 0.0, 'order': 'motorola',
// 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Throttle_report_500::Throttle_flt1Type Throttlereport500::throttle_flt1(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Throttle_report_500::Throttle_flt1Type ret =
      static_cast<Throttle_report_500::Throttle_flt1Type>(x);
  return ret;
}

// config detail: {'bit': 1, 'description': 'enable', 'enum': {0:
// 'THROTTLE_EN_STATE_MANUAL', 1: 'THROTTLE_EN_STATE_AUTO', 2:
// 'THROTTLE_EN_STATE_TAKEOVER', 3: 'THROTTLE_EN_STATE_STANDBY'},
// 'is_signed_var': False, 'len': 2, 'name': 'throttle_en_state', 'offset': 0.0,
// 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '',
// 'precision': 1.0, 'signal_type': 'enable', 'type': 'enum'}
Throttle_report_500::Throttle_en_stateType Throttlereport500::throttle_en_state(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  Throttle_report_500::Throttle_en_stateType ret =
      static_cast<Throttle_report_500::Throttle_en_stateType>(x);
  return ret;
}
}  // namespace demo
}  // namespace canbus
}  // namespace apollo
