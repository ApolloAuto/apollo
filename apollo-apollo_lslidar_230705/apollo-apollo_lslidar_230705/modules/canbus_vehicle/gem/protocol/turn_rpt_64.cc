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

#include "modules/canbus_vehicle/gem/protocol/turn_rpt_64.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Turnrpt64::Turnrpt64() {}
const int32_t Turnrpt64::ID = 0x64;

void Turnrpt64::Parse(const std::uint8_t* bytes, int32_t length,
                      Gem* chassis) const {
  chassis->mutable_turn_rpt_64()->set_manual_input(
      manual_input(bytes, length));
  chassis->mutable_turn_rpt_64()->set_commanded_value(
      commanded_value(bytes, length));
  chassis->mutable_turn_rpt_64()->set_output_value(
      output_value(bytes, length));
}

// config detail: {'name': 'manual_input', 'enum': {0: 'MANUAL_INPUT_RIGHT', 1:
// 'MANUAL_INPUT_NONE', 2: 'MANUAL_INPUT_LEFT', 3: 'MANUAL_INPUT_HAZARD'},
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Turn_rpt_64::Manual_inputType Turnrpt64::manual_input(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Turn_rpt_64::Manual_inputType ret =
      static_cast<Turn_rpt_64::Manual_inputType>(x);
  return ret;
}

// config detail: {'name': 'commanded_value', 'enum': {0:
// 'COMMANDED_VALUE_RIGHT', 1: 'COMMANDED_VALUE_NONE', 2:
// 'COMMANDED_VALUE_LEFT', 3: 'COMMANDED_VALUE_HAZARD'}, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]',
// 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Turn_rpt_64::Commanded_valueType Turnrpt64::commanded_value(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Turn_rpt_64::Commanded_valueType ret =
      static_cast<Turn_rpt_64::Commanded_valueType>(x);
  return ret;
}

// config detail: {'name': 'output_value', 'enum': {0: 'OUTPUT_VALUE_RIGHT', 1:
// 'OUTPUT_VALUE_NONE', 2: 'OUTPUT_VALUE_LEFT', 3: 'OUTPUT_VALUE_HAZARD'},
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Turn_rpt_64::Output_valueType Turnrpt64::output_value(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Turn_rpt_64::Output_valueType ret =
      static_cast<Turn_rpt_64::Output_valueType>(x);
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
