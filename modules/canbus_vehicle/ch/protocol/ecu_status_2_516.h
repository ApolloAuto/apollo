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

#include "modules/canbus_vehicle/ch/proto/ch.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ch {

class Ecustatus2516 : public ::apollo::drivers::canbus::ProtocolData<
                          ::apollo::canbus::Ch> {
 public:
  static const int32_t ID;
  Ecustatus2516();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ch* chassis) const override;

 private:
  // config detail: {'bit': 0, 'description': 'Percentage of battery remaining
  // (BMS status)', 'is_signed_var': False, 'len': 8, 'name': 'BATTERY_SOC',
  // 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]',
  // 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
  int battery_soc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 8, 'description': 'Battery full capacity (BMS
  // status)', 'is_signed_var': False, 'len': 8, 'name': 'BATTERY_CAPACITY',
  // 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]',
  // 'physical_unit': 'Ah', 'precision': 1.0, 'type': 'int'}
  int battery_capacity(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'description': 'Current battery voltage (BMS
  // status)', 'is_signed_var': False, 'len': 16, 'name': 'BATTERY_VOLTAGE',
  // 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|80]',
  // 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
  double battery_voltage(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'description': 'Current battery current (BMS
  // status)', 'is_signed_var': True, 'len': 16, 'name': 'BATTERY_CURRENT',
  // 'offset': 0.0, 'order': 'intel', 'physical_range': '[-60|60]',
  // 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  double battery_current(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 48, 'description': 'Current battery temperature (BMS
  // status)', 'is_signed_var': True, 'len': 16, 'name': 'BATTERY_TEMPERATURE',
  // 'offset': 0.0, 'order': 'intel', 'physical_range': '[-40|110]',
  // 'physical_unit': '℃', 'precision': 1.0, 'type': 'int'}
  int battery_temperature(const std::uint8_t* bytes,
                          const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
