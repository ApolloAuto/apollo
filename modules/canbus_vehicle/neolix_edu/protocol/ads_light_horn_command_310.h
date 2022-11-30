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

#include "modules/canbus_vehicle/neolix_edu/proto/neolix_edu.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

class Adslighthorncommand310 : public ::apollo::drivers::canbus::ProtocolData<
                                   ::apollo::canbus::Neolix_edu> {
 public:
  static const int32_t ID;

  Adslighthorncommand310();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': '0x0:disable ;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'Turn_Right_Light_Command',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 8, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': 'bit'}
  Adslighthorncommand310* set_turn_right_light_command(
      bool turn_right_light_command);

  // config detail: {'description': '0x0:disable ;0x1:enable ;0x2-0x3:Reserved
  // ', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name':
  // 'Turn_Left_Light_Command', 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 9, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // 'bit'}
  Adslighthorncommand310* set_turn_left_light_command(
      bool turn_left_light_command);

  // config detail: {'name': 'Horn_Command', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 10,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': 'bit'}
  Adslighthorncommand310* set_horn_command(bool horn_command);

  // config detail: {'description': '0x0:Off;0x1:LowBeam;0x2:HighBeam',
  // 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name': 'Beam_Command',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 13, 'type':
  // 'int', 'order': 'motorola', 'physical_unit': 'bit'}
  Adslighthorncommand310* set_beam_command(int beam_command);

  // config detail: {'name': 'AUTO_DriverCmd_AliveCounter', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  Adslighthorncommand310* set_auto_drivercmd_alivecounter(
      int auto_drivercmd_alivecounter);

  // config detail: {'name': 'AUTO_DriverCmd_CheckSum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  Adslighthorncommand310* set_auto_drivercmd_checksum(
      int auto_drivercmd_checksum);

 private:
  // config detail: {'description': '0x0:disable ;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'Turn_Right_Light_Command',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 8, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': 'bit'}
  void set_p_turn_right_light_command(uint8_t* data,
                                      bool turn_right_light_command);

  // config detail: {'description': '0x0:disable ;0x1:enable ;0x2-0x3:Reserved
  // ', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name':
  // 'Turn_Left_Light_Command', 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 9, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // 'bit'}
  void set_p_turn_left_light_command(uint8_t* data,
                                     bool turn_left_light_command);

  // config detail: {'name': 'Horn_Command', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 10,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': 'bit'}
  void set_p_horn_command(uint8_t* data, bool horn_command);

  // config detail: {'description': '0x0:Off;0x1:LowBeam;0x2:HighBeam',
  // 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name': 'Beam_Command',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 13, 'type':
  // 'int', 'order': 'motorola', 'physical_unit': 'bit'}
  void set_p_beam_command(uint8_t* data, int beam_command);

  // config detail: {'name': 'AUTO_DriverCmd_AliveCounter', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  void set_p_auto_drivercmd_alivecounter(uint8_t* data,
                                         int auto_drivercmd_alivecounter);

  // config detail: {'name': 'AUTO_DriverCmd_CheckSum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  void set_p_auto_drivercmd_checksum(uint8_t* data,
                                     int auto_drivercmd_checksum);

 private:
  bool turn_right_light_command_;
  bool turn_left_light_command_;
  bool horn_command_;
  int beam_command_;
  int auto_drivercmd_alivecounter_;
  int auto_drivercmd_checksum_;
};

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
