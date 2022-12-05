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

class Steeringrpt16e : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::Gem> {
 public:
  static const int32_t ID;
  Steeringrpt16e();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Gem* chassis) const override;

 private:
  // config detail: {'name': 'MANUAL_INPUT', 'offset': 0.0, 'precision': 0.001,
  // 'len': 16, 'is_signed_var': True, 'physical_range': '[-32.768|32.767]',
  // 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rad/s'}
  double manual_input(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'COMMANDED_VALUE', 'offset': 0.0, 'precision':
  // 0.001, 'len': 16, 'is_signed_var': True, 'physical_range':
  // '[-32.768|32.767]', 'bit': 23, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'rad/s'}
  double commanded_value(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'OUTPUT_VALUE', 'offset': 0.0, 'precision': 0.001,
  // 'len': 16, 'is_signed_var': True, 'physical_range': '[-32.768|32.767]',
  // 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rad/s'}
  double output_value(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
