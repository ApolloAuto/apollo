/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
namespace devkit {

class Brakecommand101 : public ::apollo::drivers::canbus::ProtocolData<
                            ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Brakecommand101();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'Brake_Dec', 'offset': 0.0, 'precision': 0.01,
  // 'len': 10, 'is_signed_var': False, 'physical_range': '[0|10.00]', 'bit':
  // 15, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  Brakecommand101* set_brake_dec(double brake_dec);

  // config detail: {'name': 'CheckSum_111', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Brakecommand101* set_checksum_111(int checksum_111);

  // config detail: {'name': 'Brake_Pedal_Target', 'offset': 0.0, 'precision':
  // 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit':
  // 31, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  Brakecommand101* set_brake_pedal_target(double brake_pedal_target);

  // config detail: {'name': 'Brake_EN_CTRL', 'enum': {0:
  // 'BRAKE_EN_CTRL_DISABLE', 1: 'BRAKE_EN_CTRL_ENABLE'}, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
  // 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Brakecommand101* set_brake_en_ctrl(
      Brake_command_101::Brake_en_ctrlType brake_en_ctrl);

 private:
  // config detail: {'name': 'Brake_Dec', 'offset': 0.0, 'precision': 0.01,
  // 'len': 10, 'is_signed_var': False, 'physical_range': '[0|10]', 'bit':
  // 15, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  void set_p_brake_dec(uint8_t* data, double brake_dec);

  // config detail: {'name': 'CheckSum_111', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_111(uint8_t* data, int checksum_111);

  // config detail: {'name': 'Brake_Pedal_Target', 'offset': 0.0, 'precision':
  // 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit':
  // 31, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  void set_p_brake_pedal_target(uint8_t* data, double brake_pedal_target);

  // config detail: {'name': 'Brake_EN_CTRL', 'enum': {0:
  // 'BRAKE_EN_CTRL_DISABLE', 1: 'BRAKE_EN_CTRL_ENABLE'}, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
  // 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_brake_en_ctrl(uint8_t* data,
                           Brake_command_101::Brake_en_ctrlType brake_en_ctrl);

 private:
  double brake_dec_;
  int checksum_111_;
  double brake_pedal_target_;
  Brake_command_101::Brake_en_ctrlType brake_en_ctrl_;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
