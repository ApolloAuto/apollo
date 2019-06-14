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

#pragma once

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ch {

class Ecustatus2516 : public ::apollo::drivers::canbus::ProtocolData<
                          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Ecustatus2516();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': 'Percentage of battery remaining (BMS
  // status)', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'name':
  // 'BATTERY_REMAINING_CAPACITY', 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
  int battery_remaining_capacity(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'description': 'Current battery voltage (BMS status)',
  // 'offset': 0.0, 'precision': 0.1, 'len': 16, 'name': 'BATTERY_VOLTAGE',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 16, 'type':
  // 'double', 'order': 'intel', 'physical_unit': 'V'}
  double battery_voltage(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Current battery current (BMS status)',
  // 'offset': 0.0, 'precision': 0.1, 'len': 16, 'name': 'BATTERY_CURRENT',
  // 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 32, 'type':
  // 'double', 'order': 'intel', 'physical_unit': 'A'}
  double battery_current(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Current battery temperature (BMS status)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 16, 'name': 'BATTERY_TEMPERATURE',
  // 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 48, 'type': 'int',
  // 'order': 'intel', 'physical_unit': '?'}
  int battery_temperature(const std::uint8_t* bytes,
                          const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
