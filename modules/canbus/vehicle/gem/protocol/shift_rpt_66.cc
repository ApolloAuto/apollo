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

#include "modules/canbus/vehicle/gem/protocol/shift_rpt_66.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Shiftrpt66::Shiftrpt66() {}
const int32_t Shiftrpt66::ID = 0x66;

void Shiftrpt66::Parse(const std::uint8_t* bytes, int32_t length,
                       ChassisDetail* chassis) const {
  chassis->mutable_gem()->mutable_shift_rpt_66()->set_manual_input(
      manual_input(bytes, length));
  chassis->mutable_gem()->mutable_shift_rpt_66()->set_commanded_value(
      commanded_value(bytes, length));
  chassis->mutable_gem()->mutable_shift_rpt_66()->set_output_value(
      output_value(bytes, length));
}

// config detail: {'name': 'manual_input', 'enum': {0: 'MANUAL_INPUT_PARK', 1:
// 'MANUAL_INPUT_REVERSE', 2: 'MANUAL_INPUT_NEUTRAL', 3: 'MANUAL_INPUT_FORWARD',
// 4: 'MANUAL_INPUT_HIGH'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 7, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Shift_rpt_66::Manual_inputType Shiftrpt66::manual_input(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Shift_rpt_66::Manual_inputType ret =
      static_cast<Shift_rpt_66::Manual_inputType>(x);
  return ret;
}

// config detail: {'name': 'commanded_value', 'enum': {0:
// 'COMMANDED_VALUE_PARK', 1: 'COMMANDED_VALUE_REVERSE', 2:
// 'COMMANDED_VALUE_NEUTRAL', 3: 'COMMANDED_VALUE_FORWARD', 4:
// 'COMMANDED_VALUE_HIGH'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 15, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Shift_rpt_66::Commanded_valueType Shiftrpt66::commanded_value(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Shift_rpt_66::Commanded_valueType ret =
      static_cast<Shift_rpt_66::Commanded_valueType>(x);
  return ret;
}

// config detail: {'name': 'output_value', 'enum': {0: 'OUTPUT_VALUE_PARK', 1:
// 'OUTPUT_VALUE_REVERSE', 2: 'OUTPUT_VALUE_NEUTRAL', 3: 'OUTPUT_VALUE_FORWARD',
// 4: 'OUTPUT_VALUE_HIGH'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 23, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Shift_rpt_66::Output_valueType Shiftrpt66::output_value(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Shift_rpt_66::Output_valueType ret =
      static_cast<Shift_rpt_66::Output_valueType>(x);
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
