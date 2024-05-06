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

#include "modules/canbus_vehicle/devkit/proto/devkit.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace devkit {

class Steeringcommand102 : public ::apollo::drivers::canbus::ProtocolData<
                               ::apollo::canbus::Devkit> {
 public:
  static const int32_t ID;

  Steeringcommand102();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Devkit* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 0, 'enum': {0: 'STEER_EN_CTRL_DISABLE', 1:
  // 'STEER_EN_CTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'Steer_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Steeringcommand102* set_steer_en_ctrl(
      Steering_command_102::Steer_en_ctrlType steer_en_ctrl);

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name':
  // 'Steer_ANGLE_Target', 'offset': -500.0, 'order': 'motorola',
  // 'physical_range': '[-500|500]''right -, left +', 'physical_unit': 'deg',
  // 'precision': 1.0, 'type': 'int'}
  Steeringcommand102* set_steer_angle_target(int steer_angle_target);

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name':
  // 'Steer_ANGLE_SPD_Target', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|250]', 'physical_unit': 'deg/s', 'precision': 1.0,
  // 'type': 'int'}
  Steeringcommand102* set_steer_angle_spd_target(int steer_angle_spd_target);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
  // 'CheckSum_102', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Steeringcommand102* set_checksum_102(int checksum_102);

 private:
  // config detail: {'bit': 0, 'enum': {0: 'STEER_EN_CTRL_DISABLE', 1:
  // 'STEER_EN_CTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'Steer_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_steer_en_ctrl(
      uint8_t* data, Steering_command_102::Steer_en_ctrlType steer_en_ctrl);

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name':
  // 'Steer_ANGLE_Target', 'offset': -500.0, 'order': 'motorola',
  // 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0,
  // 'type': 'int'}
  void set_p_steer_angle_target(uint8_t* data, int steer_angle_target);

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name':
  // 'Steer_ANGLE_SPD_Target', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|250]', 'physical_unit': 'deg/s', 'precision': 1.0,
  // 'type': 'int'}
  void set_p_steer_angle_spd_target(uint8_t* data, int steer_angle_spd_target);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
  // 'CheckSum_102', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_checksum_102(uint8_t* data, int checksum_102);

  // report the command
  Steering_command_102::Steer_en_ctrlType steer_en_ctrl(
      const std::uint8_t* bytes, const int32_t length) const;

  double steer_angle_target(const std::uint8_t* bytes,
                            const int32_t length) const;

  double steer_angle_spd_target(const std::uint8_t* bytes,
                                const int32_t length) const;

  int checksum_102(const std::uint8_t* bytes, const int32_t length) const;

 private:
  Steering_command_102::Steer_en_ctrlType steer_en_ctrl_;
  int steer_angle_target_;
  int steer_angle_spd_target_;
  int checksum_102_;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
