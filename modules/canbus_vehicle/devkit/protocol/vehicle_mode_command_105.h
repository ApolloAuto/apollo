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

#include "modules/canbus_vehicle/devkit/proto/devkit.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace devkit {

class Vehiclemodecommand105 : public ::apollo::drivers::canbus::ProtocolData<
                                  ::apollo::canbus::Devkit> {
 public:
  static const int32_t ID;

  Vehiclemodecommand105();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Devkit* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
  // 'CheckSum_105', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Vehiclemodecommand105* set_checksum_105(int checksum_105);

  // config detail: {'bit': 17, 'enum': {0: 'TURN_LIGHT_CTRL_TURNLAMP_OFF', 1:
  // 'TURN_LIGHT_CTRL_LEFT_TURNLAMP_ON', 2: 'TURN_LIGHT_CTRL_RIGHT_TURNLAMP_ON',
  // 3: 'TURN_LIGHT_CTRL_HAZARD_WARNING_LAMPSTS_ON'}, 'is_signed_var': False,
  // 'len': 2, 'name': 'Turn_Light_CTRL', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Vehiclemodecommand105* set_turn_light_ctrl(
      Vehicle_mode_command_105::Turn_light_ctrlType turn_light_ctrl);

  // config detail: {'bit': 24, 'enum': {0: 'VIN_REQ_VIN_REQ_DISABLE', 1:
  // 'VIN_REQ_VIN_REQ_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'VIN_Req', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Vehiclemodecommand105* set_vin_req(
      Vehicle_mode_command_105::Vin_reqType vin_req);

  // config detail: {'bit': 10, 'enum': {0:
  // 'DRIVE_MODE_CTRL_THROTTLE_PADDLE_DRIVE', 1: 'DRIVE_MODE_CTRL_SPEED_DRIVE'},
  // 'is_signed_var': False, 'len': 3, 'name': 'Drive_Mode_CTRL', 'offset': 0.0,
  // 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  Vehiclemodecommand105* set_drive_mode_ctrl(
      Vehicle_mode_command_105::Drive_mode_ctrlType drive_mode_ctrl);

  // config detail: {'bit': 2, 'enum': {0: 'STEER_MODE_CTRL_STANDARD_STEER', 1:
  // 'STEER_MODE_CTRL_NON_DIRECTION_STEER', 2:
  // 'STEER_MODE_CTRL_SYNC_DIRECTION_STEER'}, 'is_signed_var': False, 'len': 3,
  // 'name': 'Steer_Mode_CTRL', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Vehiclemodecommand105* set_steer_mode_ctrl(
      Vehicle_mode_command_105::Steer_mode_ctrlType steer_mode_ctrl);

 private:
  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
  // 'CheckSum_105', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_checksum_105(uint8_t* data, int checksum_105);

  // config detail: {'bit': 17, 'enum': {0: 'TURN_LIGHT_CTRL_TURNLAMP_OFF', 1:
  // 'TURN_LIGHT_CTRL_LEFT_TURNLAMP_ON', 2: 'TURN_LIGHT_CTRL_RIGHT_TURNLAMP_ON',
  // 3: 'TURN_LIGHT_CTRL_HAZARD_WARNING_LAMPSTS_ON'}, 'is_signed_var': False,
  // 'len': 2, 'name': 'Turn_Light_CTRL', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  void set_p_turn_light_ctrl(
      uint8_t* data,
      Vehicle_mode_command_105::Turn_light_ctrlType turn_light_ctrl);

  // config detail: {'bit': 24, 'enum': {0: 'VIN_REQ_VIN_REQ_DISABLE', 1:
  // 'VIN_REQ_VIN_REQ_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'VIN_Req', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_vin_req(uint8_t* data,
                     Vehicle_mode_command_105::Vin_reqType vin_req);

  // config detail: {'bit': 10, 'enum': {0:
  // 'DRIVE_MODE_CTRL_THROTTLE_PADDLE_DRIVE', 1: 'DRIVE_MODE_CTRL_SPEED_DRIVE'},
  // 'is_signed_var': False, 'len': 3, 'name': 'Drive_Mode_CTRL', 'offset': 0.0,
  // 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  void set_p_drive_mode_ctrl(
      uint8_t* data,
      Vehicle_mode_command_105::Drive_mode_ctrlType drive_mode_ctrl);

  // config detail: {'bit': 2, 'enum': {0: 'STEER_MODE_CTRL_STANDARD_STEER', 1:
  // 'STEER_MODE_CTRL_NON_DIRECTION_STEER', 2:
  // 'STEER_MODE_CTRL_SYNC_DIRECTION_STEER'}, 'is_signed_var': False, 'len': 3,
  // 'name': 'Steer_Mode_CTRL', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  void set_p_steer_mode_ctrl(
      uint8_t* data,
      Vehicle_mode_command_105::Steer_mode_ctrlType steer_mode_ctrl);

  // report the command
  Vehicle_mode_command_105::Turn_light_ctrlType turn_light_ctrl(
      const std::uint8_t* bytes, const int32_t length) const;

  Vehicle_mode_command_105::Vin_reqType vin_req(const std::uint8_t* bytes,
                                                const int32_t length) const;

  Vehicle_mode_command_105::Drive_mode_ctrlType drive_mode_ctrl(
      const std::uint8_t* bytes, const int32_t length) const;

  Vehicle_mode_command_105::Steer_mode_ctrlType steer_mode_ctrl(
      const std::uint8_t* bytes, const int32_t length) const;

  int checksum_105(const std::uint8_t* bytes, const int32_t length) const;

 private:
  int checksum_105_;
  Vehicle_mode_command_105::Turn_light_ctrlType turn_light_ctrl_;
  Vehicle_mode_command_105::Vin_reqType vin_req_;
  Vehicle_mode_command_105::Drive_mode_ctrlType drive_mode_ctrl_;
  Vehicle_mode_command_105::Steer_mode_ctrlType steer_mode_ctrl_;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
