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

class Gearcommand103
    : public ::apollo::drivers::canbus::ProtocolData<::apollo::canbus::Demo> {
 public:
  static const int32_t ID;

  Gearcommand103();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Demo* chassis) const override;

  void UpdateData_Heartbeat(uint8_t* data) override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 4, 'name':
  // 'Heartbeat_103', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Gearcommand103* set_heartbeat_103(int heartbeat_103);

  // config detail: {'bit': 10, 'description': 'command', 'enum': {0:
  // 'GEAR_TARGET_INVALID', 1: 'GEAR_TARGET_PARK', 2: 'GEAR_TARGET_REVERSE', 3:
  // 'GEAR_TARGET_NEUTRAL', 4: 'GEAR_TARGET_DRIVE'}, 'is_signed_var': False,
  // 'len': 3, 'name': 'Gear_Target', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|4]', 'physical_unit': '', 'precision': 1.0,
  // 'signal_type': 'command', 'type': 'enum'}
  Gearcommand103* set_gear_target(
      Gear_command_103::Gear_targetType gear_target);

  // config detail: {'bit': 0, 'description': 'enable', 'enum': {0:
  // 'GEAR_EN_CTRL_DISABLE', 1: 'GEAR_EN_CTRL_ENABLE'}, 'is_signed_var': False,
  // 'len': 1, 'name': 'Gear_EN_CTRL', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0,
  // 'signal_type': 'enable', 'type': 'enum'}
  Gearcommand103* set_gear_en_ctrl(
      Gear_command_103::Gear_en_ctrlType gear_en_ctrl);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
  // 'CheckSum_103', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Gearcommand103* set_checksum_103(int checksum_103);

 private:
  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 4, 'name':
  // 'Heartbeat_103', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_heartbeat_103(uint8_t* data, int heartbeat_103);

  // config detail: {'bit': 10, 'description': 'command', 'enum': {0:
  // 'GEAR_TARGET_INVALID', 1: 'GEAR_TARGET_PARK', 2: 'GEAR_TARGET_REVERSE', 3:
  // 'GEAR_TARGET_NEUTRAL', 4: 'GEAR_TARGET_DRIVE'}, 'is_signed_var': False,
  // 'len': 3, 'name': 'Gear_Target', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|4]', 'physical_unit': '', 'precision': 1.0,
  // 'signal_type': 'command', 'type': 'enum'}
  void set_p_gear_target(uint8_t* data,
                         Gear_command_103::Gear_targetType gear_target);

  // config detail: {'bit': 0, 'description': 'enable', 'enum': {0:
  // 'GEAR_EN_CTRL_DISABLE', 1: 'GEAR_EN_CTRL_ENABLE'}, 'is_signed_var': False,
  // 'len': 1, 'name': 'Gear_EN_CTRL', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0,
  // 'signal_type': 'enable', 'type': 'enum'}
  void set_p_gear_en_ctrl(uint8_t* data,
                          Gear_command_103::Gear_en_ctrlType gear_en_ctrl);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
  // 'CheckSum_103', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_checksum_103(uint8_t* data, int checksum_103);

  int heartbeat_103(const std::uint8_t* bytes, const int32_t length) const;

  Gear_command_103::Gear_targetType gear_target(const std::uint8_t* bytes,
                                                const int32_t length) const;

  Gear_command_103::Gear_en_ctrlType gear_en_ctrl(const std::uint8_t* bytes,
                                                  const int32_t length) const;

  int checksum_103(const std::uint8_t* bytes, const int32_t length) const;

 private:
  int heartbeat_103_;
  Gear_command_103::Gear_targetType gear_target_;
  Gear_command_103::Gear_en_ctrlType gear_en_ctrl_;
  int checksum_103_;
};

}  // namespace demo
}  // namespace canbus
}  // namespace apollo
