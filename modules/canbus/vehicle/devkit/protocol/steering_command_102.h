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

class Steeringcommand102 : public ::apollo::drivers::canbus::ProtocolData<
                               ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Steeringcommand102();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'Steer_EN_CTRL', 'enum': {0:
  // 'STEER_EN_CTRL_DISABLE', 1: 'STEER_EN_CTRL_ENABLE'}, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
  // 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Steeringcommand102* set_steer_en_ctrl(
      Steering_command_102::Steer_en_ctrlType steer_en_ctrl);

  // config detail: {'name': 'Steer_ANGLE_Target', 'offset': -500.0,
  // 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[-500|500]', 'bit': 31, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'deg'}
  Steeringcommand102* set_steer_angle_target(double steer_angle_target);

  // config detail: {'name': 'Steer_ANGLE_SPD', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|250]', 'bit': 15,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg/s'}
  Steeringcommand102* set_steer_angle_spd(int steer_angle_spd);

  // config detail: {'name': 'CheckSum_102', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Steeringcommand102* set_checksum_102(int checksum_102);

 private:
  // config detail: {'name': 'Steer_EN_CTRL', 'enum': {0:
  // 'STEER_EN_CTRL_DISABLE', 1: 'STEER_EN_CTRL_ENABLE'}, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
  // 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_steer_en_ctrl(
      uint8_t* data, Steering_command_102::Steer_en_ctrlType steer_en_ctrl);

  // config detail: {'name': 'Steer_ANGLE_Target', 'offset': -500.0,
  // 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[-500|500]', 'bit': 31, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'deg'}
  void set_p_steer_angle_target(uint8_t* data, double steer_angle_target);

  // config detail: {'name': 'Steer_ANGLE_SPD', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|250]', 'bit': 15,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg/s'}
  void set_p_steer_angle_spd(uint8_t* data, int steer_angle_spd);

  // config detail: {'name': 'CheckSum_102', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_102(uint8_t* data, int checksum_102);

 private:
  Steering_command_102::Steer_en_ctrlType steer_en_ctrl_;
  double steer_angle_target_;
  int steer_angle_spd_;
  int checksum_102_;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
