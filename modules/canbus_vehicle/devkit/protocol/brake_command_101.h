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

class Brakecommand101 : public ::apollo::drivers::canbus::ProtocolData<
                            ::apollo::canbus::Devkit> {
 public:
  static const int32_t ID;

  Brakecommand101();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Devkit* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 0, 'enum': {0: 'BRAKE_EN_CTRL_DISABLE', 1:
  // 'BRAKE_EN_CTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'Brake_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Brakecommand101* set_brake_en_ctrl(
      Brake_command_101::Brake_en_ctrlType brake_en_ctrl);

  // config detail: {'bit': 1, 'enum': {0: 'AEB_EN_CTRL_DISABLE_AEB', 1:
  // 'AEB_EN_CTRL_ENABLE_AEB'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'AEB_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Brakecommand101* set_aeb_en_ctrl(
      Brake_command_101::Aeb_en_ctrlType aeb_en_ctrl);

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 10, 'name':
  // 'Brake_Dec', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|10]', 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
  Brakecommand101* set_brake_dec(double brake_dec);

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name':
  // 'Brake_Pedal_Target', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  Brakecommand101* set_brake_pedal_target(double brake_pedal_target);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
  // 'CheckSum_101', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Brakecommand101* set_checksum_101(int checksum_101);

 private:
  // config detail: {'bit': 0, 'enum': {0: 'BRAKE_EN_CTRL_DISABLE', 1:
  // 'BRAKE_EN_CTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'Brake_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_brake_en_ctrl(uint8_t* data,
                           Brake_command_101::Brake_en_ctrlType brake_en_ctrl);

  // config detail: {'bit': 1, 'enum': {0: 'AEB_EN_CTRL_DISABLE_AEB', 1:
  // 'AEB_EN_CTRL_ENABLE_AEB'}, 'is_signed_var': False, 'len': 1, 'name':
  // 'AEB_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_aeb_en_ctrl(uint8_t* data,
                         Brake_command_101::Aeb_en_ctrlType aeb_en_ctrl);

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 10, 'name':
  // 'Brake_Dec', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|10]', 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
  void set_p_brake_dec(uint8_t* data, double brake_dec);

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name':
  // 'Brake_Pedal_Target', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  void set_p_brake_pedal_target(uint8_t* data, double brake_pedal_target);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
  // 'CheckSum_101', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_checksum_101(uint8_t* data, int checksum_101);

  // report the command
  Brake_command_101::Brake_en_ctrlType brake_en_ctrl(
      const std::uint8_t* bytes, const int32_t length) const;

  Brake_command_101::Aeb_en_ctrlType aeb_en_ctrl(const std::uint8_t* bytes,
                                                 const int32_t length) const;

  double brake_dec(const std::uint8_t* bytes, const int32_t length) const;

  double brake_pedal_target(const std::uint8_t* bytes,
                            const int32_t length) const;

  int checksum_101(const std::uint8_t* bytes, const int32_t length) const;

 private:
  Brake_command_101::Aeb_en_ctrlType aeb_en_ctrl_;
  double brake_dec_;
  int checksum_101_;
  double brake_pedal_target_;
  Brake_command_101::Brake_en_ctrlType brake_en_ctrl_;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
