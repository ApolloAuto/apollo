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

#include "modules/canbus_vehicle/demo/protocol/park_report_504.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace demo {

using ::apollo::drivers::canbus::Byte;

Parkreport504::Parkreport504() {}
const int32_t Parkreport504::ID = 0x504;

void Parkreport504::Parse(const std::uint8_t* bytes, int32_t length,
                          Demo* chassis) const {
  chassis->mutable_park_report_504()->set_parking_actual(
      parking_actual(bytes, length));
  chassis->mutable_park_report_504()->set_park_flt(park_flt(bytes, length));
}

// config detail: {'bit': 0, 'description': 'command', 'enum': {0:
// 'PARKING_ACTUAL_RELEASE', 1: 'PARKING_ACTUAL_PARKING_TRIGGER'},
// 'is_signed_var': False, 'len': 1, 'name': 'parking_actual', 'offset': 0.0,
// 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '',
// 'precision': 1.0, 'signal_type': 'command', 'type': 'enum'}
Park_report_504::Parking_actualType Parkreport504::parking_actual(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Park_report_504::Parking_actualType ret =
      static_cast<Park_report_504::Parking_actualType>(x);
  return ret;
}

// config detail: {'bit': 15, 'description': 'fault', 'enum': {0:
// 'PARK_FLT_NO_FAULT', 1: 'PARK_FLT_FAULT'}, 'is_signed_var': False, 'len': 8,
// 'name': 'park_flt', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Park_report_504::Park_fltType Parkreport504::park_flt(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Park_report_504::Park_fltType ret =
      static_cast<Park_report_504::Park_fltType>(x);
  return ret;
}
}  // namespace demo
}  // namespace canbus
}  // namespace apollo
