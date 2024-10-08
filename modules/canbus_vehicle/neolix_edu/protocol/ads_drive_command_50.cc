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

#include "modules/canbus_vehicle/neolix_edu/protocol/ads_drive_command_50.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

const int32_t Adsdrivecommand50::ID = 0x50;

// public
Adsdrivecommand50::Adsdrivecommand50() { Reset(); }

uint32_t Adsdrivecommand50::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Adsdrivecommand50::Parse(const std::uint8_t* bytes, int32_t length,
                              Neolix_edu* chassis) const {
  chassis->mutable_ads_drive_command_50()->set_drive_enable(
      drive_enable(bytes, length));
  chassis->mutable_ads_drive_command_50()->set_auto_shift_command(
      auto_shift_command(bytes, length));
  chassis->mutable_ads_drive_command_50()->set_auto_drive_torque(
      auto_drive_torque(bytes, length));
}

void Adsdrivecommand50::UpdateData(uint8_t* data) {
  set_p_drive_enable(data, drive_enable_);
  set_p_auto_shift_command(data, auto_shift_command_);
  set_p_auto_drive_torque(data, auto_drive_torque_);
  ++auto_drivercmd_alivecounter_;
  auto_drivercmd_alivecounter_ = (auto_drivercmd_alivecounter_) % 16;
  set_p_auto_drivercmd_alivecounter(data, auto_drivercmd_alivecounter_);
  auto_drivercmd_checksum_ =
      data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
  set_p_auto_drivercmd_checksum(data, auto_drivercmd_checksum_);
}

void Adsdrivecommand50::Reset() {
  // TODO(All) :  you should check this manually
  drive_enable_ = false;
  auto_shift_command_ = Ads_drive_command_50::AUTO_SHIFT_COMMAND_N;
  auto_drive_torque_ = 0.0;
  auto_drivercmd_alivecounter_ = 0;
  auto_drivercmd_checksum_ = 0;
}

Adsdrivecommand50* Adsdrivecommand50::set_drive_enable(bool drive_enable) {
  drive_enable_ = drive_enable;
  return this;
}

// config detail: {'description': '0x0:disable ;0x1:enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'Drive_Enable', 'is_signed_var': False,
// 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': ''}
void Adsdrivecommand50::set_p_drive_enable(uint8_t* data, bool drive_enable) {
  int x = drive_enable;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

bool Adsdrivecommand50::drive_enable(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

Adsdrivecommand50* Adsdrivecommand50::set_auto_shift_command(
    Ads_drive_command_50::Auto_shift_commandType auto_shift_command) {
  auto_shift_command_ = auto_shift_command;
  return this;
}

// config detail: {'description': '0x0:N ;0x1:D ;0x2:R ;0x3:Reserved ', 'enum':
// {0: 'AUTO_SHIFT_COMMAND_N', 1: 'AUTO_SHIFT_COMMAND_D', 2:
// 'AUTO_SHIFT_COMMAND_R', 3: 'AUTO_SHIFT_COMMAND_RESERVED'}, 'precision': 1.0,
// 'len': 2, 'name': 'AUTO_Shift_Command', 'is_signed_var': False, 'offset':
// 0.0, 'physical_range': '[0|3]', 'bit': 9, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
void Adsdrivecommand50::set_p_auto_shift_command(
    uint8_t* data,
    Ads_drive_command_50::Auto_shift_commandType auto_shift_command) {
  int x = auto_shift_command;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 2);
}

Ads_drive_command_50::Auto_shift_commandType
Adsdrivecommand50::auto_shift_command(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 2);

  Ads_drive_command_50::Auto_shift_commandType ret =
      static_cast<Ads_drive_command_50::Auto_shift_commandType>(x);
  return ret;
}

Adsdrivecommand50* Adsdrivecommand50::set_auto_drive_torque(
    double auto_drive_torque) {
  auto_drive_torque_ = auto_drive_torque;
  return this;
}

// config detail: {'name': 'AUTO_Drive_Torque', 'offset': -665.0, 'precision':
// 0.02, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 23, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
void Adsdrivecommand50::set_p_auto_drive_torque(uint8_t* data,
                                                double auto_drive_torque) {
  auto_drive_torque = ProtocolData::BoundedValue(0.0, 50.0, auto_drive_torque);
  int x = (auto_drive_torque - -665.000000) / 0.020000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 3);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 2);
  to_set1.set_value(t, 0, 8);
}

double Adsdrivecommand50::auto_drive_torque(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = (x * 0.020000) + -665.000000;
  return ret;
}

Adsdrivecommand50* Adsdrivecommand50::set_auto_drivercmd_alivecounter(
    int auto_drivercmd_alivecounter) {
  auto_drivercmd_alivecounter_ = auto_drivercmd_alivecounter;
  return this;
}

// config detail: {'name': 'AUTO_DriverCmd_AliveCounter', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Adsdrivecommand50::set_p_auto_drivercmd_alivecounter(
    uint8_t* data, int auto_drivercmd_alivecounter) {
  auto_drivercmd_alivecounter =
      ProtocolData::BoundedValue(0, 15, auto_drivercmd_alivecounter);
  int x = auto_drivercmd_alivecounter;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 4);
}

Adsdrivecommand50* Adsdrivecommand50::set_auto_drivercmd_checksum(
    int auto_drivercmd_checksum) {
  auto_drivercmd_checksum_ = auto_drivercmd_checksum;
  return this;
}

// config detail: {'name': 'AUTO_DriverCmd_CheckSum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Adsdrivecommand50::set_p_auto_drivercmd_checksum(
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
