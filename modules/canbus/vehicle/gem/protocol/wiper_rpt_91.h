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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace gem {

class Wiperrpt91 : public ::apollo::drivers::canbus::ProtocolData<
                       ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Wiperrpt91();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'OUTPUT_VALUE', 'enum': {0:
  // 'OUTPUT_VALUE_WIPERS_OFF', 1: 'OUTPUT_VALUE_INTERMITTENT_1', 2:
  // 'OUTPUT_VALUE_INTERMITTENT_2', 3: 'OUTPUT_VALUE_INTERMITTENT_3', 4:
  // 'OUTPUT_VALUE_INTERMITTENT_4', 5: 'OUTPUT_VALUE_INTERMITTENT_5', 6:
  // 'OUTPUT_VALUE_LOW', 7: 'OUTPUT_VALUE_HIGH'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit':
  // 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Wiper_rpt_91::Output_valueType output_value(const std::uint8_t* bytes,
                                              const int32_t length) const;

  // config detail: {'name': 'COMMANDED_VALUE', 'enum': {0:
  // 'COMMANDED_VALUE_WIPERS_OFF', 1: 'COMMANDED_VALUE_INTERMITTENT_1', 2:
  // 'COMMANDED_VALUE_INTERMITTENT_2', 3: 'COMMANDED_VALUE_INTERMITTENT_3', 4:
  // 'COMMANDED_VALUE_INTERMITTENT_4', 5: 'COMMANDED_VALUE_INTERMITTENT_5', 6:
  // 'COMMANDED_VALUE_LOW', 7: 'COMMANDED_VALUE_HIGH'}, 'precision': 1.0, 'len':
  // 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit':
  // 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Wiper_rpt_91::Commanded_valueType commanded_value(const std::uint8_t* bytes,
                                                    const int32_t length) const;

  // config detail: {'name': 'MANUAL_INPUT', 'enum': {0:
  // 'MANUAL_INPUT_WIPERS_OFF', 1: 'MANUAL_INPUT_INTERMITTENT_1', 2:
  // 'MANUAL_INPUT_INTERMITTENT_2', 3: 'MANUAL_INPUT_INTERMITTENT_3', 4:
  // 'MANUAL_INPUT_INTERMITTENT_4', 5: 'MANUAL_INPUT_INTERMITTENT_5', 6:
  // 'MANUAL_INPUT_LOW', 7: 'MANUAL_INPUT_HIGH'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 7,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Wiper_rpt_91::Manual_inputType manual_input(const std::uint8_t* bytes,
                                              const int32_t length) const;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
