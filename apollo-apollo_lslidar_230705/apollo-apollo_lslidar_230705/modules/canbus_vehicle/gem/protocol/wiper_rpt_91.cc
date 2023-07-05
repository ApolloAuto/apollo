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

#include "modules/canbus_vehicle/gem/protocol/wiper_rpt_91.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Wiperrpt91::Wiperrpt91() {}
const int32_t Wiperrpt91::ID = 0x91;

void Wiperrpt91::Parse(const std::uint8_t* bytes, int32_t length,
                       Gem* chassis) const {
  chassis->mutable_wiper_rpt_91()->set_output_value(
      output_value(bytes, length));
  chassis->mutable_wiper_rpt_91()->set_commanded_value(
      commanded_value(bytes, length));
  chassis->mutable_wiper_rpt_91()->set_manual_input(
      manual_input(bytes, length));
}

// config detail: {'name': 'output_value', 'enum': {0:
// 'OUTPUT_VALUE_WIPERS_OFF', 1: 'OUTPUT_VALUE_INTERMITTENT_1', 2:
// 'OUTPUT_VALUE_INTERMITTENT_2', 3: 'OUTPUT_VALUE_INTERMITTENT_3', 4:
// 'OUTPUT_VALUE_INTERMITTENT_4', 5: 'OUTPUT_VALUE_INTERMITTENT_5', 6:
// 'OUTPUT_VALUE_LOW', 7: 'OUTPUT_VALUE_HIGH'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 23,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Wiper_rpt_91::Output_valueType Wiperrpt91::output_value(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Wiper_rpt_91::Output_valueType ret =
      static_cast<Wiper_rpt_91::Output_valueType>(x);
  return ret;
}

// config detail: {'name': 'commanded_value', 'enum': {0:
// 'COMMANDED_VALUE_WIPERS_OFF', 1: 'COMMANDED_VALUE_INTERMITTENT_1', 2:
// 'COMMANDED_VALUE_INTERMITTENT_2', 3: 'COMMANDED_VALUE_INTERMITTENT_3', 4:
// 'COMMANDED_VALUE_INTERMITTENT_4', 5: 'COMMANDED_VALUE_INTERMITTENT_5', 6:
// 'COMMANDED_VALUE_LOW', 7: 'COMMANDED_VALUE_HIGH'}, 'precision': 1.0, 'len':
// 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit':
// 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Wiper_rpt_91::Commanded_valueType Wiperrpt91::commanded_value(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Wiper_rpt_91::Commanded_valueType ret =
      static_cast<Wiper_rpt_91::Commanded_valueType>(x);
  return ret;
}

// config detail: {'name': 'manual_input', 'enum': {0:
// 'MANUAL_INPUT_WIPERS_OFF', 1: 'MANUAL_INPUT_INTERMITTENT_1', 2:
// 'MANUAL_INPUT_INTERMITTENT_2', 3: 'MANUAL_INPUT_INTERMITTENT_3', 4:
// 'MANUAL_INPUT_INTERMITTENT_4', 5: 'MANUAL_INPUT_INTERMITTENT_5', 6:
// 'MANUAL_INPUT_LOW', 7: 'MANUAL_INPUT_HIGH'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 7,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Wiper_rpt_91::Manual_inputType Wiperrpt91::manual_input(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Wiper_rpt_91::Manual_inputType ret =
      static_cast<Wiper_rpt_91::Manual_inputType>(x);
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
