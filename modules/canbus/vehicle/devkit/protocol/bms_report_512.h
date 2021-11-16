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
namespace devkit {

class Bmsreport512 : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Bmsreport512();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 23, 'description': 'Battery Total Current',
  // 'is_signed_var': False, 'len': 16, 'name': 'Battery_Current', 'offset':
  // -3200.0, 'order': 'motorola', 'physical_range': '[-3200|3200]',
  // 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  double battery_current(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 7, 'description': 'Battery Total Voltage',
  // 'is_signed_var': False, 'len': 16, 'name': 'Battery_Voltage', 'offset':
  // 0.0, 'order': 'motorola', 'physical_range': '[0|300]', 'physical_unit':
  // 'V', 'precision': 0.01, 'type': 'double'}
  double battery_voltage(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 39, 'description': 'Battery Soc percentage',
  // 'is_signed_var': False, 'len': 8, 'name': 'Battery_Soc', 'offset': 0.0,
  // 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%',
  // 'precision': 1.0, 'type': 'int'}
  int battery_soc(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
