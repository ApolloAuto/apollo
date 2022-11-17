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

#include "modules/canbus/vehicle/neolix_edu/protocol/ads_eps_command_56.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

const int32_t Adsepscommand56::ID = 0x56;

// public
Adsepscommand56::Adsepscommand56() { Reset(); }

uint32_t Adsepscommand56::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Adsepscommand56::UpdateData(uint8_t* data) {
  set_p_drive_enable(data, drive_enable_);
  set_p_auto_target_angle(data, auto_target_angle_);
  ++auto_drivercmd_alivecounter_;
  auto_drivercmd_alivecounter_ = (auto_drivercmd_alivecounter_) % 16;
  set_p_auto_drivercmd_alivecounter(data, auto_drivercmd_alivecounter_);
  auto_drivercmd_checksum_ =
      data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
  set_p_auto_drivercmd_checksum(data, auto_drivercmd_checksum_);
}

void Adsepscommand56::Reset() {
  // TODO(All) :  you should check this manually
  drive_enable_ = false;
  auto_target_angle_ = 0.0;
  auto_drivercmd_alivecounter_ = 0;
  auto_drivercmd_checksum_ = 0;
}

Adsepscommand56* Adsepscommand56::set_drive_enable(bool drive_enable) {
  drive_enable_ = drive_enable;
  return this;
}

// config detail: {'description': '0x0:disable ;0x1:enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'Drive_Enable', 'is_signed_var': False,
// 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': ''}
void Adsepscommand56::set_p_drive_enable(uint8_t* data, bool drive_enable) {
  int x = drive_enable;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

Adsepscommand56* Adsepscommand56::set_auto_target_angle(
    double auto_target_angle) {
  auto_target_angle_ = -auto_target_angle;
  return this;
}

// config detail: {'name': 'AUTO_Target_Angle', 'offset': -2048.0, 'precision':
// 0.0625, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 23, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
void Adsepscommand56::set_p_auto_target_angle(uint8_t* data,
                                              double auto_target_angle) {
  auto_target_angle =
      ProtocolData::BoundedValue(-380.0, 380.0, auto_target_angle);
  int x = (auto_target_angle - -2048.000000) / 0.062500;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 3);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 2);
  to_set1.set_value(t, 0, 8);
}

Adsepscommand56* Adsepscommand56::set_auto_drivercmd_alivecounter(
    int auto_drivercmd_alivecounter) {
  auto_drivercmd_alivecounter_ = auto_drivercmd_alivecounter;
  return this;
}

// config detail: {'name': 'AUTO_DriverCmd_AliveCounter', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Adsepscommand56::set_p_auto_drivercmd_alivecounter(
    uint8_t* data, int auto_drivercmd_alivecounter) {
  auto_drivercmd_alivecounter =
      ProtocolData::BoundedValue(0, 15, auto_drivercmd_alivecounter);
  int x = auto_drivercmd_alivecounter;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 4);
}

Adsepscommand56* Adsepscommand56::set_auto_drivercmd_checksum(
    int auto_drivercmd_checksum) {
  auto_drivercmd_checksum_ = auto_drivercmd_checksum;
  return this;
}

// config detail: {'name': 'AUTO_DriverCmd_CheckSum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Adsepscommand56::set_p_auto_drivercmd_checksum(
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
