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
  // config detail: {'description': 'Current speed (Steering status)', 'offset':
  // 0.0, 'precision': 0.01, 'len': 16, 'name': 'SPEED', 'is_signed_var': True,
  // 'physical_range': '[0|0]', 'bit': 0, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'm/s'}
  double speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Current acceleration (Steering status)',
  // 'offset': 0.0, 'precision': 0.001, 'len': 16, 'name': 'ACC_SPEED',
  // 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 16, 'type':
  // 'double', 'order': 'intel', 'physical_unit': 'm/s^2'}
  double acc_speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Current Auto-mode state (Chassis status)',
  // 'enum': {0: 'CTRL_STS_OUT_OF_CONTROL', 1: 'CTRL_STS_UNDER_CONTROL'},
  // 'precision': 1.0, 'len': 8, 'name': 'CTRL_STS', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum',
  // 'order': 'intel', 'physical_unit': ''}
  Ecu_status_1_515::Ctrl_stsType ctrl_sts(const std::uint8_t* bytes,
                                          const int32_t length) const;

  // config detail: {'description': 'Current chassis state (Chassis status)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name': 'CHASSIS_STS',
  // 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 40, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  int chassis_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Chassis error code (Chassis status)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 16, 'name': 'CHASSIS_ERR',
  // 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 48, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  int chassis_err(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
