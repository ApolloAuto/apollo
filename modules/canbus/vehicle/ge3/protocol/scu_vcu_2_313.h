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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ge3 {

class Scuvcu2313 : public ::apollo::drivers::canbus::ProtocolData<
                       ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Scuvcu2313();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': 'Max positive torque', 'offset': 0.0,
  // 'precision': 1.5, 'len': 11, 'name': 'VCU_TorqPosMax', 'is_signed_var':
  // False, 'physical_range': '[0|3000]', 'bit': 55, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'Nm'}
  double vcu_torqposmax(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Max negative torque', 'offset': -3000.0,
  // 'precision': 1.5, 'len': 11, 'name': 'VCU_TorqNegMax', 'is_signed_var':
  // False, 'physical_range': '[-3000|0]', 'bit': 39, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'Nm'}
  double vcu_torqnegmax(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Actual torque', 'offset': -3000.0,
  // 'precision': 1.5, 'len': 12, 'name': 'VCU_TorqAct', 'is_signed_var': False,
  // 'physical_range': '[-3000|3000]', 'bit': 23, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'Nm'}
  double vcu_torqact(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Engine speed', 'offset': 0.0,
  // 'precision': 1.0, 'len': 16, 'name': 'VCU_EngSpd', 'is_signed_var': False,
  // 'physical_range': '[0|65535]', 'bit': 7, 'type': 'int', 'order':
  // 'motorola', 'physical_unit': 'rpm'}
  int vcu_engspd(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
