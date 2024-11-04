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

#include "modules/canbus_vehicle/demo/protocol/gear_report_503.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace demo {

using ::apollo::drivers::canbus::Byte;

Gearreport503::Gearreport503() {}
const int32_t Gearreport503::ID = 0x503;

void Gearreport503::Parse(const std::uint8_t* bytes, int32_t length,
                          Demo* chassis) const {
  chassis->mutable_gear_report_503()->set_gear_flt(gear_flt(bytes, length));
  chassis->mutable_gear_report_503()->set_gear_actual(
      gear_actual(bytes, length));
}

// config detail: {'bit': 15, 'description': 'fault', 'enum': {0:
// 'GEAR_FLT_NO_FAULT', 1: 'GEAR_FLT_FAULT'}, 'is_signed_var': False, 'len': 8,
// 'name': 'gear_flt', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Gear_report_503::Gear_fltType Gearreport503::gear_flt(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Gear_report_503::Gear_fltType ret =
      static_cast<Gear_report_503::Gear_fltType>(x);
  return ret;
}

// config detail: {'bit': 2, 'description': 'command', 'enum': {0:
// 'GEAR_ACTUAL_INVALID', 1: 'GEAR_ACTUAL_PARK', 2: 'GEAR_ACTUAL_REVERSE', 3:
// 'GEAR_ACTUAL_NEUTRAL', 4: 'GEAR_ACTUAL_DRIVE'}, 'is_signed_var': False,
// 'len': 3, 'name': 'gear_actual', 'offset': 0.0, 'order': 'motorola',
// 'physical_range': '[0|4]', 'physical_unit': '', 'precision': 1.0,
// 'signal_type': 'command', 'type': 'enum'}
Gear_report_503::Gear_actualType Gearreport503::gear_actual(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 3);

  Gear_report_503::Gear_actualType ret =
      static_cast<Gear_report_503::Gear_actualType>(x);
  return ret;
}
}  // namespace demo
}  // namespace canbus
}  // namespace apollo
