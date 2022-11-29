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

#include "modules/canbus_vehicle/devkit/protocol/park_report_504.h"

#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::drivers::canbus::Byte;

Parkreport504::Parkreport504() {}
const int32_t Parkreport504::ID = 0x504;

void Parkreport504::Parse(const std::uint8_t* bytes, int32_t length,
                          Devkit* chassis) const {
  chassis->mutable_park_report_504()->set_parking_actual(
      parking_actual(bytes, length));
  chassis->mutable_park_report_504()->set_park_flt(
      park_flt(bytes, length));
}

// config detail: {'name': 'parking_actual', 'enum': {0:
// 'PARKING_ACTUAL_RELEASE', 1: 'PARKING_ACTUAL_PARKING_TRIGGER'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Park_report_504::Parking_actualType Parkreport504::parking_actual(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Park_report_504::Parking_actualType ret =
      static_cast<Park_report_504::Parking_actualType>(x);
  return ret;
}

// config detail: {'name': 'park_flt', 'enum': {0: 'PARK_FLT_NO_FAULT', 1:
// 'PARK_FLT_FAULT'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Park_report_504::Park_fltType Parkreport504::park_flt(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Park_report_504::Park_fltType ret =
      static_cast<Park_report_504::Park_fltType>(x);
  return ret;
}
}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
