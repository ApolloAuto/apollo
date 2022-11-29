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

#include "modules/canbus_vehicle/gem/proto/gem.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace gem {

class Shiftrpt66 : public ::apollo::drivers::canbus::ProtocolData<
                       ::apollo::canbus::Gem> {
 public:
  static const int32_t ID;
  Shiftrpt66();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Gem* chassis) const override;

 private:
  // config detail: {'name': 'MANUAL_INPUT', 'enum': {0: 'MANUAL_INPUT_PARK', 1:
  // 'MANUAL_INPUT_REVERSE', 2: 'MANUAL_INPUT_NEUTRAL', 3:
  // 'MANUAL_INPUT_FORWARD', 4: 'MANUAL_INPUT_HIGH'}, 'precision': 1.0, 'len':
  // 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit':
  // 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Shift_rpt_66::Manual_inputType manual_input(const std::uint8_t* bytes,
                                              const int32_t length) const;

  // config detail: {'name': 'COMMANDED_VALUE', 'enum': {0:
  // 'COMMANDED_VALUE_PARK', 1: 'COMMANDED_VALUE_REVERSE', 2:
  // 'COMMANDED_VALUE_NEUTRAL', 3: 'COMMANDED_VALUE_FORWARD', 4:
  // 'COMMANDED_VALUE_HIGH'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 15, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Shift_rpt_66::Commanded_valueType commanded_value(const std::uint8_t* bytes,
                                                    const int32_t length) const;

  // config detail: {'name': 'OUTPUT_VALUE', 'enum': {0: 'OUTPUT_VALUE_PARK', 1:
  // 'OUTPUT_VALUE_REVERSE', 2: 'OUTPUT_VALUE_NEUTRAL', 3:
  // 'OUTPUT_VALUE_FORWARD', 4: 'OUTPUT_VALUE_HIGH'}, 'precision': 1.0, 'len':
  // 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit':
  // 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Shift_rpt_66::Output_valueType output_value(const std::uint8_t* bytes,
                                              const int32_t length) const;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
