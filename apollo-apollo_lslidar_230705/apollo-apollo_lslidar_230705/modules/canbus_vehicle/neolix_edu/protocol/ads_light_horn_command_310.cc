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

#include "modules/canbus_vehicle/neolix_edu/protocol/ads_light_horn_command_310.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

const int32_t Adslighthorncommand310::ID = 0x310;

// public
Adslighthorncommand310::Adslighthorncommand310() { Reset(); }

uint32_t Adslighthorncommand310::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Adslighthorncommand310::UpdateData(uint8_t* data) {
  set_p_turn_right_light_command(data, turn_right_light_command_);
  set_p_turn_left_light_command(data, turn_left_light_command_);
  set_p_horn_command(data, horn_command_);
  set_p_beam_command(data, beam_command_);
  ++auto_drivercmd_alivecounter_;
  auto_drivercmd_alivecounter_ = (auto_drivercmd_alivecounter_) % 16;
  set_p_auto_drivercmd_alivecounter(data, auto_drivercmd_alivecounter_);
  auto_drivercmd_checksum_ =
      data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
  set_p_auto_drivercmd_checksum(data, auto_drivercmd_checksum_);
}

void Adslighthorncommand310::Reset() {
  // TODO(All) :  you should check this manually
  turn_right_light_command_ = false;
  turn_left_light_command_ = false;
  horn_command_ = false;
  beam_command_ = 0;
  auto_drivercmd_alivecounter_ = 0;
  auto_drivercmd_checksum_ = 0;
}

Adslighthorncommand310* Adslighthorncommand310::set_turn_right_light_command(
    bool turn_right_light_command) {
  turn_right_light_command_ = turn_right_light_command;
  return this;
}

// config detail: {'description': '0x0:disable ;0x1:enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'Turn_Right_Light_Command',
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 8, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': 'bit'}
void Adslighthorncommand310::set_p_turn_right_light_command(
    uint8_t* data, bool turn_right_light_command) {
  int x = turn_right_light_command;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 1);
}

Adslighthorncommand310* Adslighthorncommand310::set_turn_left_light_command(
    bool turn_left_light_command) {
  turn_left_light_command_ = turn_left_light_command;
  return this;
}

// config detail: {'description': '0x0:disable ;0x1:enable ;0x2-0x3:Reserved ',
// 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'Turn_Left_Light_Command',
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 9, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': 'bit'}
void Adslighthorncommand310::set_p_turn_left_light_command(
    uint8_t* data, bool turn_left_light_command) {
  int x = turn_left_light_command;

  Byte to_set(data + 1);
  to_set.set_value(x, 1, 1);
}

Adslighthorncommand310* Adslighthorncommand310::set_horn_command(
    bool horn_command) {
  horn_command_ = horn_command;
  return this;
}

// config detail: {'name': 'Horn_Command', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 10,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': 'bit'}
void Adslighthorncommand310::set_p_horn_command(uint8_t* data,
                                                bool horn_command) {
  int x = horn_command;

  Byte to_set(data + 1);
  to_set.set_value(x, 2, 1);
}

Adslighthorncommand310* Adslighthorncommand310::set_beam_command(
    int beam_command) {
  beam_command_ = beam_command;
  return this;
}

// config detail: {'description': '0x0:Off;0x1:LowBeam;0x2:HighBeam', 'offset':
// 0.0, 'precision': 1.0, 'len': 2, 'name': 'Beam_Command', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 13, 'type': 'int', 'order':
// 'motorola', 'physical_unit': 'bit'}
void Adslighthorncommand310::set_p_beam_command(uint8_t* data,
                                                int beam_command) {
  beam_command = ProtocolData::BoundedValue(0, 1, beam_command);
  int x = beam_command;

  Byte to_set(data + 1);
  to_set.set_value(x, 4, 2);
}

Adslighthorncommand310* Adslighthorncommand310::set_auto_drivercmd_alivecounter(
    int auto_drivercmd_alivecounter) {
  auto_drivercmd_alivecounter_ = auto_drivercmd_alivecounter;
  return this;
}

// config detail: {'name': 'AUTO_DriverCmd_AliveCounter', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Adslighthorncommand310::set_p_auto_drivercmd_alivecounter(
    uint8_t* data, int auto_drivercmd_alivecounter) {
  auto_drivercmd_alivecounter =
      ProtocolData::BoundedValue(0, 15, auto_drivercmd_alivecounter);
  int x = auto_drivercmd_alivecounter;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 4);
}

Adslighthorncommand310* Adslighthorncommand310::set_auto_drivercmd_checksum(
    int auto_drivercmd_checksum) {
  auto_drivercmd_checksum_ = auto_drivercmd_checksum;
  return this;
}

// config detail: {'name': 'AUTO_DriverCmd_CheckSum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Adslighthorncommand310::set_p_auto_drivercmd_checksum(
    uint8_t* data, int auto_drivercmd_checksum) {
  auto_drivercmd_checksum =
      ProtocolData::BoundedValue(0, 255, auto_drivercmd_checksum);
  int x = auto_drivercmd_checksum;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
