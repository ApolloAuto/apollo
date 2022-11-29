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

#pragma once

#include "modules/canbus_vehicle/lexus/proto/lexus.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace lexus {

class Datetimerpt40f : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::Lexus> {
 public:
  static const int32_t ID;
  Datetimerpt40f();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Lexus* chassis) const override;

 private:
  // config detail: {'name': 'TIME_SECOND', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|60]', 'bit': 47,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': 'sec'}
  int time_second(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TIME_MINUTE', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|60]', 'bit': 39,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': 'min'}
  int time_minute(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TIME_HOUR', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|23]', 'bit': 31,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': 'hr'}
  int time_hour(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'DATE_DAY', 'offset': 1.0, 'precision': 1.0, 'len':
  // 8, 'is_signed_var': False, 'physical_range': '[1|31]', 'bit': 23, 'type':
  // 'int', 'order': 'motorola', 'physical_unit': 'dy'}
  int date_day(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'DATE_MONTH', 'offset': 1.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[1|12]', 'bit': 15,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': 'mon'}
  int date_month(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'DATE_YEAR', 'offset': 2000.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[2000|2255]', 'bit':
  // 7, 'type': 'int', 'order': 'motorola', 'physical_unit': 'yr'}
  int date_year(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
