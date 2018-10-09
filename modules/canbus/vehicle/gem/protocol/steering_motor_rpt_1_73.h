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

class Steeringmotorrpt173 : public ::apollo::drivers::canbus::ProtocolData<
                                ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Steeringmotorrpt173();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'MOTOR_CURRENT', 'offset': 0.0, 'precision': 0.001,
  // 'len': 32, 'is_signed_var': False, 'physical_range': '[0|4294967.295]',
  // 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'amps'}
  double motor_current(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SHAFT_POSITION', 'offset': 0.0, 'precision':
  // 0.001, 'len': 32, 'is_signed_var': True, 'physical_range':
  // '[-2147483.648|2147483.647]', 'bit': 39, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'amps'}
  double shaft_position(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
