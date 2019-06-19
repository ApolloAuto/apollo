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

class Steercommand112 : public ::apollo::drivers::canbus::ProtocolData<
                            ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Steercommand112();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': 'steering angle enable bit(Command)',
  // 'enum': {0: 'STEER_ANGLE_EN_CTRL_DISABLE', 1:
  // 'STEER_ANGLE_EN_CTRL_ENABLE'}, 'precision': 1.0, 'len': 8, 'name':
  // 'STEER_ANGLE_EN_CTRL', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  Steercommand112* set_steer_angle_en_ctrl(
      Steer_command_112::Steer_angle_en_ctrlType steer_angle_en_ctrl);

  // config detail: {'description': 'Current steering angle(Command)', 'offset':
  // 0.0, 'precision': 0.001, 'len': 16, 'name': 'STEER_ANGLE_CMD',
  // 'is_signed_var': True, 'physical_range': '[-0.524|0.524]', 'bit': 8,
  // 'type': 'double', 'order': 'intel', 'physical_unit': 'radian'}
  Steercommand112* set_steer_angle_cmd(double steer_angle_cmd);

 private:
  // config detail: {'description': 'steering angle enable bit(Command)',
  // 'enum': {0: 'STEER_ANGLE_EN_CTRL_DISABLE', 1:
  // 'STEER_ANGLE_EN_CTRL_ENABLE'}, 'precision': 1.0, 'len': 8, 'name':
  // 'STEER_ANGLE_EN_CTRL', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  void set_p_steer_angle_en_ctrl(
      uint8_t* data,
      Steer_command_112::Steer_angle_en_ctrlType steer_angle_en_ctrl);

  // config detail: {'description': 'Current steering angle(Command)', 'offset':
  // 0.0, 'precision': 0.001, 'len': 16, 'name': 'STEER_ANGLE_CMD',
  // 'is_signed_var': True, 'physical_range': '[-0.524|0.524]', 'bit': 8,
  // 'type': 'double', 'order': 'intel', 'physical_unit': 'radian'}
  void set_p_steer_angle_cmd(uint8_t* data, double steer_angle_cmd);

 private:
  Steer_command_112::Steer_angle_en_ctrlType steer_angle_en_ctrl_;
  double steer_angle_cmd_;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
