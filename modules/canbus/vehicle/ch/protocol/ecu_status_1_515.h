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
namespace ch {

class Ecustatus1515 : public ::apollo::drivers::canbus::ProtocolData<
                          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Ecustatus1515();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 0, 'description': 'Current speed (Steering status)',
  // 'is_signed_var': True, 'len': 16, 'name': 'SPEED', 'offset': 0.0, 'order':
  // 'intel', 'physical_range': '[-327.68|327.67]', 'physical_unit': 'm/s',
  // 'precision': 0.01, 'type': 'double'}
  double speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'description': 'Current acceleration (Steering
  // status)', 'is_signed_var': True, 'len': 16, 'name': 'ACC_SPEED', 'offset':
  // 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'm/s^2',
  // 'precision': 0.001, 'type': 'double'}
  double acc_speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'description': 'Current Auto-mode state (Chassis
  // status)', 'enum': {0: 'CTRL_STS_OUT_OF_CONTROL', 1:
  // 'CTRL_STS_UNDER_CONTROL'}, 'is_signed_var': False, 'len': 8, 'name':
  // 'CTRL_STS', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Ecu_status_1_515::Ctrl_stsType ctrl_sts(const std::uint8_t* bytes,
                                          const int32_t length) const;

  // config detail: {'bit': 40, 'description': 'Current chassis state (Chassis
  // status)', 'is_signed_var': False, 'len': 8, 'name': 'CHASSIS_STS',
  // 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassis_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 48, 'description': 'Chassis error code (Chassis
  // status)', 'is_signed_var': False, 'len': 16, 'name': 'CHASSIS_ERR',
  // 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassis_err(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
