/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/demo/proto/demo.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace demo {

class Parkcommand104
    : public ::apollo::drivers::canbus::ProtocolData<::apollo::canbus::Demo> {
 public:
  static const int32_t ID;

  Parkcommand104();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Demo* chassis) const override;

  void UpdateData_Heartbeat(uint8_t* data) override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 4, 'name':
  // 'Heartbeat_104', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Parkcommand104* set_heartbeat_104(int heartbeat_104);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
  // 'CheckSum_104', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Parkcommand104* set_checksum_104(int checksum_104);

  // config detail: {'bit': 8, 'description': 'command', 'enum': {0:
  // 'PARK_TARGET_RELEASE', 1: 'PARK_TARGET_PARKING_TRIGGER'}, 'is_signed_var':
  // False, 'len': 1, 'name': 'Park_Target', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0,
  // 'signal_type': 'command', 'type': 'enum'}
  Parkcommand104* set_park_target(
      Park_command_104::Park_targetType park_target);

  // config detail: {'bit': 0, 'description': 'enable', 'enum': {0:
  // 'PARK_EN_CTRL_DISABLE', 1: 'PARK_EN_CTRL_ENABLE'}, 'is_signed_var': False,
  // 'len': 1, 'name': 'Park_EN_CTRL', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0,
  // 'signal_type': 'enable', 'type': 'enum'}
  Parkcommand104* set_park_en_ctrl(
      Park_command_104::Park_en_ctrlType park_en_ctrl);

 private:
  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 4, 'name':
  // 'Heartbeat_104', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_heartbeat_104(uint8_t* data, int heartbeat_104);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
  // 'CheckSum_104', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_checksum_104(uint8_t* data, int checksum_104);

  // config detail: {'bit': 8, 'description': 'command', 'enum': {0:
  // 'PARK_TARGET_RELEASE', 1: 'PARK_TARGET_PARKING_TRIGGER'}, 'is_signed_var':
  // False, 'len': 1, 'name': 'Park_Target', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0,
  // 'signal_type': 'command', 'type': 'enum'}
  void set_p_park_target(uint8_t* data,
                         Park_command_104::Park_targetType park_target);

  // config detail: {'bit': 0, 'description': 'enable', 'enum': {0:
  // 'PARK_EN_CTRL_DISABLE', 1: 'PARK_EN_CTRL_ENABLE'}, 'is_signed_var': False,
  // 'len': 1, 'name': 'Park_EN_CTRL', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0,
  // 'signal_type': 'enable', 'type': 'enum'}
  void set_p_park_en_ctrl(uint8_t* data,
                          Park_command_104::Park_en_ctrlType park_en_ctrl);

  int heartbeat_104(const std::uint8_t* bytes, const int32_t length) const;

  int checksum_104(const std::uint8_t* bytes, const int32_t length) const;

  Park_command_104::Park_targetType park_target(const std::uint8_t* bytes,
                                                const int32_t length) const;

  Park_command_104::Park_en_ctrlType park_en_ctrl(const std::uint8_t* bytes,
                                                  const int32_t length) const;

 private:
  int heartbeat_104_;
  int checksum_104_;
  Park_command_104::Park_targetType park_target_;
  Park_command_104::Park_en_ctrlType park_en_ctrl_;
};

}  // namespace demo
}  // namespace canbus
}  // namespace apollo
