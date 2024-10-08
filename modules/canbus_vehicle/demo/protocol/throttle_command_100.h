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

class Throttlecommand100
    : public ::apollo::drivers::canbus::ProtocolData<::apollo::canbus::Demo> {
 public:
  static const int32_t ID;

  Throttlecommand100();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Demo* chassis) const override;

  void UpdateData_Heartbeat(uint8_t* data) override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 4, 'name':
  // 'Heartbeat_100', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Throttlecommand100* set_heartbeat_100(int heartbeat_100);

  // config detail: {'bit': 47, 'is_signed_var': False, 'len': 12, 'name':
  // 'Speed_Target', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|40.95]', 'physical_unit': 'm/s', 'precision': 0.01, 'type': 'double'}
  Throttlecommand100* set_speed_target(double speed_target);

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 10, 'name':
  // 'Throttle_Acc', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|10]', 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
  Throttlecommand100* set_throttle_acc(double throttle_acc);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
  // 'CheckSum_100', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Throttlecommand100* set_checksum_100(int checksum_100);

  // config detail: {'bit': 31, 'description': 'command', 'is_signed_var':
  // False, 'len': 16, 'name': 'Throttle_Pedal_Target', 'offset': 0.0, 'order':
  // 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision':
  // 0.1, 'signal_type': 'command', 'type': 'double'}
  Throttlecommand100* set_throttle_pedal_target(double throttle_pedal_target);

  // config detail: {'bit': 0, 'description': 'enable', 'enum': {0:
  // 'THROTTLE_EN_CTRL_DISABLE', 1: 'THROTTLE_EN_CTRL_ENABLE'}, 'is_signed_var':
  // False, 'len': 1, 'name': 'Throttle_EN_CTRL', 'offset': 0.0, 'order':
  // 'motorola', 'physical_range': '[0|1]', 'physical_unit': '',
  // 'precision': 1.0, 'signal_type': 'enable', 'type': 'enum'}
  Throttlecommand100* set_throttle_en_ctrl(
      Throttle_command_100::Throttle_en_ctrlType throttle_en_ctrl);

 private:
  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 4, 'name':
  // 'Heartbeat_100', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_heartbeat_100(uint8_t* data, int heartbeat_100);

  // config detail: {'bit': 47, 'is_signed_var': False, 'len': 12, 'name':
  // 'Speed_Target', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|40.95]', 'physical_unit': 'm/s', 'precision': 0.01, 'type': 'double'}
  void set_p_speed_target(uint8_t* data, double speed_target);

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 10, 'name':
  // 'Throttle_Acc', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|10]', 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
  void set_p_throttle_acc(uint8_t* data, double throttle_acc);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
  // 'CheckSum_100', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_checksum_100(uint8_t* data, int checksum_100);

  // config detail: {'bit': 31, 'description': 'command', 'is_signed_var':
  // False, 'len': 16, 'name': 'Throttle_Pedal_Target', 'offset': 0.0, 'order':
  // 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision':
  // 0.1, 'signal_type': 'command', 'type': 'double'}
  void set_p_throttle_pedal_target(uint8_t* data, double throttle_pedal_target);

  // config detail: {'bit': 0, 'description': 'enable', 'enum': {0:
  // 'THROTTLE_EN_CTRL_DISABLE', 1: 'THROTTLE_EN_CTRL_ENABLE'}, 'is_signed_var':
  // False, 'len': 1, 'name': 'Throttle_EN_CTRL', 'offset': 0.0, 'order':
  // 'motorola', 'physical_range': '[0|1]', 'physical_unit': '',
  // 'precision': 1.0, 'signal_type': 'enable', 'type': 'enum'}
  void set_p_throttle_en_ctrl(
      uint8_t* data,
      Throttle_command_100::Throttle_en_ctrlType throttle_en_ctrl);

  int heartbeat_100(const std::uint8_t* bytes, const int32_t length) const;

  double speed_target(const std::uint8_t* bytes, const int32_t length) const;

  double throttle_acc(const std::uint8_t* bytes, const int32_t length) const;

  int checksum_100(const std::uint8_t* bytes, const int32_t length) const;

  double throttle_pedal_target(const std::uint8_t* bytes,
                               const int32_t length) const;

  Throttle_command_100::Throttle_en_ctrlType throttle_en_ctrl(
      const std::uint8_t* bytes, const int32_t length) const;

 private:
  int heartbeat_100_;
  double speed_target_;
  double throttle_acc_;
  int checksum_100_;
  double throttle_pedal_target_;
  Throttle_command_100::Throttle_en_ctrlType throttle_en_ctrl_;
};

}  // namespace demo
}  // namespace canbus
}  // namespace apollo
