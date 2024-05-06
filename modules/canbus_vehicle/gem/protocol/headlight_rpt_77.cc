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

#include "modules/canbus_vehicle/gem/protocol/headlight_rpt_77.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Headlightrpt77::Headlightrpt77() {}
const int32_t Headlightrpt77::ID = 0x77;

void Headlightrpt77::Parse(const std::uint8_t* bytes, int32_t length,
                           Gem* chassis) const {
  chassis->mutable_headlight_rpt_77()->set_output_value(
      output_value(bytes, length));
  chassis->mutable_headlight_rpt_77()->set_manual_input(
      manual_input(bytes, length));
  chassis->mutable_headlight_rpt_77()->set_commanded_value(
      commanded_value(bytes, length));
}

// config detail: {'name': 'output_value', 'enum': {0:
// 'OUTPUT_VALUE_HEADLIGHTS_OFF', 1: 'OUTPUT_VALUE_LOW_BEAMS', 2:
// 'OUTPUT_VALUE_HIGH_BEAMS'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 23, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Headlight_rpt_77::Output_valueType Headlightrpt77::output_value(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Headlight_rpt_77::Output_valueType ret =
      static_cast<Headlight_rpt_77::Output_valueType>(x);
  return ret;
}

// config detail: {'name': 'manual_input', 'enum': {0:
// 'MANUAL_INPUT_HEADLIGHTS_OFF', 1: 'MANUAL_INPUT_LOW_BEAMS', 2:
// 'MANUAL_INPUT_HIGH_BEAMS'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 7, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Headlight_rpt_77::Manual_inputType Headlightrpt77::manual_input(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Headlight_rpt_77::Manual_inputType ret =
      static_cast<Headlight_rpt_77::Manual_inputType>(x);
  return ret;
}

// config detail: {'name': 'commanded_value', 'enum': {0:
// 'COMMANDED_VALUE_HEADLIGHTS_OFF', 1: 'COMMANDED_VALUE_LOW_BEAMS', 2:
// 'COMMANDED_VALUE_HIGH_BEAMS'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 15, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Headlight_rpt_77::Commanded_valueType Headlightrpt77::commanded_value(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Headlight_rpt_77::Commanded_valueType ret =
      static_cast<Headlight_rpt_77::Commanded_valueType>(x);
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
