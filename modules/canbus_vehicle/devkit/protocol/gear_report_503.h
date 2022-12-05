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

#pragma once

#include "modules/canbus_vehicle/devkit/proto/devkit.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace devkit {

class Gearreport503 : public ::apollo::drivers::canbus::ProtocolData<
                          ::apollo::canbus::Devkit> {
 public:
  static const int32_t ID;
  Gearreport503();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Devkit* chassis) const override;

 private:
  // config detail: {'name': 'Gear_FLT', 'enum': {0: 'GEAR_FLT_NO_FAULT', 1:
  // 'GEAR_FLT_FAULT'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Gear_report_503::Gear_fltType gear_flt(const std::uint8_t* bytes,
                                         const int32_t length) const;

  // config detail: {'name': 'Gear_Actual', 'enum': {0: 'GEAR_ACTUAL_INVALID',
  // 1: 'GEAR_ACTUAL_PARK', 2: 'GEAR_ACTUAL_REVERSE', 3: 'GEAR_ACTUAL_NEUTRAL',
  // 4: 'GEAR_ACTUAL_DRIVE'}, 'precision': 1.0, 'len': 3, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 2, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Gear_report_503::Gear_actualType gear_actual(const std::uint8_t* bytes,
                                               const int32_t length) const;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
