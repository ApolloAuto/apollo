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

class Throttlereport500 : public ::apollo::drivers::canbus::ProtocolData<
                              ::apollo::canbus::Devkit> {
 public:
  static const int32_t ID;
  Throttlereport500();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Devkit* chassis) const override;

 private:
  // config detail: {'name': 'Throttle_Pedal_Actual', 'offset': 0.0,
  // 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[0|100]', 'bit': 31, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': '%'}
  double throttle_pedal_actual(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'description': 'Drive system communication fault', 'enum':
  // {0: 'THROTTLE_FLT2_NO_FAULT', 1:
  // 'THROTTLE_FLT2_DRIVE_SYSTEM_COMUNICATION_FAULT'}, 'precision': 1.0, 'len':
  // 8, 'name': 'Throttle_FLT2', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Throttle_report_500::Throttle_flt2Type throttle_flt2(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Drive system hardware fault', 'enum': {0:
  // 'THROTTLE_FLT1_NO_FAULT', 1: 'THROTTLE_FLT1_DRIVE_SYSTEM_HARDWARE_FAULT'},
  // 'precision': 1.0, 'len': 8, 'name': 'Throttle_FLT1', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Throttle_report_500::Throttle_flt1Type throttle_flt1(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Throttle_EN_state', 'enum': {0:
  // 'THROTTLE_EN_STATE_MANUAL', 1: 'THROTTLE_EN_STATE_AUTO', 2:
  // 'THROTTLE_EN_STATE_TAKEOVER', 3: 'THROTTLE_EN_STATE_STANDBY'},
  // 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|2]', 'bit': 1, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Throttle_report_500::Throttle_en_stateType throttle_en_state(
      const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
