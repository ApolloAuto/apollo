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

#pragma once

#include "modules/canbus_vehicle/demo/proto/demo.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace demo {

class Parkreport504
    : public ::apollo::drivers::canbus::ProtocolData<::apollo::canbus::Demo> {
 public:
  static const int32_t ID;
  Parkreport504();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Demo* chassis) const override;

 private:
  // config detail: {'bit': 0, 'description': 'command', 'enum': {0:
  // 'PARKING_ACTUAL_RELEASE', 1: 'PARKING_ACTUAL_PARKING_TRIGGER'},
  // 'is_signed_var': False, 'len': 1, 'name': 'Parking_actual', 'offset': 0.0,
  // 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '',
  // 'precision': 1.0, 'signal_type': 'command', 'type': 'enum'}
  Park_report_504::Parking_actualType parking_actual(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 15, 'description': 'fault', 'enum': {0:
  // 'PARK_FLT_NO_FAULT', 1: 'PARK_FLT_FAULT'}, 'is_signed_var': False, 'len':
  // 8, 'name': 'Park_FLT', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Park_report_504::Park_fltType park_flt(const std::uint8_t* bytes,
                                         const int32_t length) const;
};

}  // namespace demo
}  // namespace canbus
}  // namespace apollo
