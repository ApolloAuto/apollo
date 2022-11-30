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

class Parkcommand104 : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::Devkit> {
 public:
  static const int32_t ID;

  Parkcommand104();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Devkit* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'CheckSum_104', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Parkcommand104* set_checksum_104(int checksum_104);

  // config detail: {'name': 'Park_Target', 'enum': {0: 'PARK_TARGET_RELEASE',
  // 1: 'PARK_TARGET_PARKING_TRIGGER'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Parkcommand104* set_park_target(
      Park_command_104::Park_targetType park_target);

  // config detail: {'name': 'Park_EN_CTRL', 'enum': {0: 'PARK_EN_CTRL_DISABLE',
  // 1: 'PARK_EN_CTRL_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Parkcommand104* set_park_en_ctrl(
      Park_command_104::Park_en_ctrlType park_en_ctrl);

 private:
  // config detail: {'name': 'CheckSum_104', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_104(uint8_t* data, int checksum_104);

  // config detail: {'name': 'Park_Target', 'enum': {0: 'PARK_TARGET_RELEASE',
  // 1: 'PARK_TARGET_PARKING_TRIGGER'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_park_target(uint8_t* data,
                         Park_command_104::Park_targetType park_target);

  // config detail: {'name': 'Park_EN_CTRL', 'enum': {0: 'PARK_EN_CTRL_DISABLE',
  // 1: 'PARK_EN_CTRL_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_park_en_ctrl(uint8_t* data,
                          Park_command_104::Park_en_ctrlType park_en_ctrl);

  // report the command
  Park_command_104::Park_targetType park_target(const std::uint8_t* bytes,
                                                const int32_t length) const;

  Park_command_104::Park_en_ctrlType park_en_ctrl(const std::uint8_t* bytes,
                                                  const int32_t length) const;

  int checksum_104(const std::uint8_t* bytes, const int32_t length) const;

 private:
  int checksum_104_;
  Park_command_104::Park_targetType park_target_;
  Park_command_104::Park_en_ctrlType park_en_ctrl_;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
