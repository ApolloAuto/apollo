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

#include "modules/canbus_vehicle/zhongyun/proto/zhongyun.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

class Vehiclestatefeedback2c4 : public ::apollo::drivers::canbus::ProtocolData<
                                    ::apollo::canbus::Zhongyun> {
 public:
  static const int32_t ID;
  Vehiclestatefeedback2c4();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Zhongyun* chassis) const override;

 private:
  // config detail: {'name': 'motor_speed', 'offset': 0.0, 'precision': 1.0,
  // 'len': 16, 'is_signed_var': True, 'physical_range': '[-3000|3000]', 'bit':
  // 0, 'type': 'int', 'order': 'intel', 'physical_unit': 'rpm'}
  int motor_speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'driven_torque_feedback', 'offset': 0.0,
  // 'precision': 0.05, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[0|100]', 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit':
  // '%'}
  double driven_torque_feedback(const std::uint8_t* bytes,
                                const int32_t length) const;
};

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
