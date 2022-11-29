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

class Gearcommand103 : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::Devkit> {
 public:
  static const int32_t ID;

  Gearcommand103();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Devkit* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'Gear_Target', 'enum': {0: 'GEAR_TARGET_INVALID',
  // 1: 'GEAR_TARGET_PARK', 2: 'GEAR_TARGET_REVERSE', 3: 'GEAR_TARGET_NEUTRAL',
  // 4: 'GEAR_TARGET_DRIVE'}, 'precision': 1.0, 'len': 3, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 10, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Gearcommand103* set_gear_target(
      Gear_command_103::Gear_targetType gear_target);

  // config detail: {'name': 'Gear_EN_CTRL', 'enum': {0: 'GEAR_EN_CTRL_DISABLE',
  // 1: 'GEAR_EN_CTRL_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Gearcommand103* set_gear_en_ctrl(
      Gear_command_103::Gear_en_ctrlType gear_en_ctrl);

  // config detail: {'name': 'CheckSum_103', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Gearcommand103* set_checksum_103(int checksum_103);

 private:
  // config detail: {'name': 'Gear_Target', 'enum': {0: 'GEAR_TARGET_INVALID',
  // 1: 'GEAR_TARGET_PARK', 2: 'GEAR_TARGET_REVERSE', 3: 'GEAR_TARGET_NEUTRAL',
  // 4: 'GEAR_TARGET_DRIVE'}, 'precision': 1.0, 'len': 3, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 10, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_gear_target(uint8_t* data,
                         Gear_command_103::Gear_targetType gear_target);

  // config detail: {'name': 'Gear_EN_CTRL', 'enum': {0: 'GEAR_EN_CTRL_DISABLE',
  // 1: 'GEAR_EN_CTRL_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_gear_en_ctrl(uint8_t* data,
                          Gear_command_103::Gear_en_ctrlType gear_en_ctrl);

  // config detail: {'name': 'CheckSum_103', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_103(uint8_t* data, int checksum_103);

  // report the command
  Gear_command_103::Gear_targetType gear_target(const std::uint8_t* bytes,
                                                const int32_t length) const;

  Gear_command_103::Gear_en_ctrlType gear_en_ctrl(const std::uint8_t* bytes,
                                                  const int32_t length) const;

  int checksum_103(const std::uint8_t* bytes, const int32_t length) const;

 private:
  Gear_command_103::Gear_targetType gear_target_;
  Gear_command_103::Gear_en_ctrlType gear_en_ctrl_;
  int checksum_103_;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
