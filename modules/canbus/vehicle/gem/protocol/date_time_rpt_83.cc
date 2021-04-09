/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/gem/protocol/date_time_rpt_83.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Datetimerpt83::Datetimerpt83() {}
const int32_t Datetimerpt83::ID = 0x83;

void Datetimerpt83::Parse(const std::uint8_t* bytes, int32_t length,
                          ChassisDetail* chassis) const {
  chassis->mutable_gem()->mutable_date_time_rpt_83()->set_time_second(
      time_second(bytes, length));
  chassis->mutable_gem()->mutable_date_time_rpt_83()->set_time_minute(
      time_minute(bytes, length));
  chassis->mutable_gem()->mutable_date_time_rpt_83()->set_time_hour(
      time_hour(bytes, length));
  chassis->mutable_gem()->mutable_date_time_rpt_83()->set_date_day(
      date_day(bytes, length));
  chassis->mutable_gem()->mutable_date_time_rpt_83()->set_date_month(
      date_month(bytes, length));
  chassis->mutable_gem()->mutable_date_time_rpt_83()->set_date_year(
      date_year(bytes, length));
}

// config detail: {'name': 'time_second', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|60]', 'bit': 47,
// 'type': 'int', 'order': 'motorola', 'physical_unit': 'sec'}
int Datetimerpt83::time_second(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'time_minute', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|60]', 'bit': 39,
// 'type': 'int', 'order': 'motorola', 'physical_unit': 'min'}
int Datetimerpt83::time_minute(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'time_hour', 'offset': 0.0, 'precision': 1.0, 'len':
// 8, 'is_signed_var': False, 'physical_range': '[0|23]', 'bit': 31, 'type':
// 'int', 'order': 'motorola', 'physical_unit': 'hr'}
int Datetimerpt83::time_hour(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'date_day', 'offset': 1.0, 'precision': 1.0, 'len':
// 8, 'is_signed_var': False, 'physical_range': '[1|31]', 'bit': 23, 'type':
// 'int', 'order': 'motorola', 'physical_unit': 'dy'}
int Datetimerpt83::date_day(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + 1.000000;
  return ret;
}

// config detail: {'name': 'date_month', 'offset': 1.0, 'precision': 1.0, 'len':
// 8, 'is_signed_var': False, 'physical_range': '[1|12]', 'bit': 15, 'type':
// 'int', 'order': 'motorola', 'physical_unit': 'mon'}
int Datetimerpt83::date_month(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + 1.000000;
  return ret;
}

// config detail: {'name': 'date_year', 'offset': 2000.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[2000|2255]', 'bit': 7,
// 'type': 'int', 'order': 'motorola', 'physical_unit': 'yr'}
int Datetimerpt83::date_year(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + 2000.000000;
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
