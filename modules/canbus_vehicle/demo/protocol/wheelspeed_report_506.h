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

class Wheelspeedreport506
    : public ::apollo::drivers::canbus::ProtocolData<::apollo::canbus::Demo> {
 public:
  static const int32_t ID;
  Wheelspeedreport506();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Demo* chassis) const override;

 private:
  // config detail: {'bit': 55, 'is_signed_var': False, 'len': 16, 'name': 'RR',
  // 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65.535]',
  // 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double rr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 39, 'is_signed_var': False, 'len': 16, 'name': 'RL',
  // 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65.535]',
  // 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double rl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 23, 'is_signed_var': False, 'len': 16, 'name': 'FR',
  // 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65.535]',
  // 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double fr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 16, 'name': 'FL',
  // 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65.535]',
  // 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double fl(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace demo
}  // namespace canbus
}  // namespace apollo
