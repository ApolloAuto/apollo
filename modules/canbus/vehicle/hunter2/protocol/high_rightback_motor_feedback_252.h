/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

class Highrightbackmotorfeedback252 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Highrightbackmotorfeedback252();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'bit': 39, 'is_signed_var': True, 'len': 32, 'name': 'high_motor_position', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': 'PN', 'precision': 1.0, 'type': 'int'}
  int high_motor_position(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'high_motor_current', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-3276.8|3276.7]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  double high_motor_current(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 7, 'is_signed_var': True, 'len': 16, 'name': 'high_motor_rpm', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32768|32767]', 'physical_unit': 'RPM', 'precision': 1.0, 'type': 'int'}
  int high_motor_rpm(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo


