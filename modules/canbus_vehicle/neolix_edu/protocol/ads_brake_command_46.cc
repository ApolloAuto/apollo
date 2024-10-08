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

#include "modules/canbus_vehicle/neolix_edu/protocol/ads_brake_command_46.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

const int32_t Adsbrakecommand46::ID = 0x46;

// public
Adsbrakecommand46::Adsbrakecommand46() { Reset(); }

uint32_t Adsbrakecommand46::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Adsbrakecommand46::Parse(const std::uint8_t* bytes, int32_t length,
                              Neolix_edu* chassis) const {
  chassis->mutable_ads_brake_command_46()->set_drive_enable(
      drive_enable(bytes, length));
  chassis->mutable_ads_brake_command_46()->set_auto_brake_command(
      auto_brake_command(bytes, length));
  chassis->mutable_ads_brake_command_46()->set_auto_parking_command(
      auto_parking_command(bytes, length));
}

void Adsbrakecommand46::UpdateData(uint8_t* data) {
  set_p_drive_enable(data, drive_enable_);
  set_p_auto_brake_command(data, auto_brake_command_);
  set_p_auto_parking_command(data, auto_parking_command_);
  set_p_epb_rampauxiliarycommand(data, epb_rampauxiliarycommand_);
  ++auto_drivercmd_alivecounter_;
  auto_drivercmd_alivecounter_ = (auto_drivercmd_alivecounter_) % 16;
  set_p_auto_drivercmd_alivecounter(data, auto_drivercmd_alivecounter_);
  auto_drivercmd_checksum_ =
      data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
  set_p_auto_drivercmd_checksum(data, auto_drivercmd_checksum_);
}

void Adsbrakecommand46::Reset() {
  // TODO(All) :  you should check this manually
  drive_enable_ = false;
  auto_brake_command_ = 0;
  auto_parking_command_ = false;
  epb_rampauxiliarycommand_ = false;
  auto_drivercmd_alivecounter_ = 0;
  auto_drivercmd_checksum_ = 0;
}

Adsbrakecommand46* Adsbrakecommand46::set_drive_enable(bool drive_enable) {
  drive_enable_ = drive_enable;
  return this;
}

// config detail: {'description': '0x0:disable ;0x1:enable', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'Drive_Enable', 'is_signed_var': False,
// 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order': 'motorola',
// 'physical_unit': ''}
void Adsbrakecommand46::set_p_drive_enable(uint8_t* data, bool drive_enable) {
  int x = drive_enable;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

bool Adsbrakecommand46::drive_enable(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

Adsbrakecommand46* Adsbrakecommand46::set_auto_brake_command(
    int auto_brake_command) {
  auto_brake_command_ = auto_brake_command;
  return this;
}

// config detail: {'name': 'AUTO_Brake_Command', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Adsbrakecommand46::set_p_auto_brake_command(uint8_t* data,
                                                 int auto_brake_command) {
  auto_brake_command = ProtocolData::BoundedValue(0, 100, auto_brake_command);
  int x = auto_brake_command;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 8);
}

int32_t Adsbrakecommand46::auto_brake_command(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int32_t ret = x;
  return ret;
}

Adsbrakecommand46* Adsbrakecommand46::set_auto_parking_command(
    bool auto_parking_command) {
  auto_parking_command_ = auto_parking_command;
  return this;
}

// config detail: {'description': '0x0:Release ;0x1:Apply ', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'AUTO_Parking_Command', 'is_signed_var':
// False, 'physical_range': '[0|0]', 'bit': 24, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': ''}
void Adsbrakecommand46::set_p_auto_parking_command(uint8_t* data,
                                                   bool auto_parking_command) {
  int x = auto_parking_command;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 1);
}

bool Adsbrakecommand46::auto_parking_command(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

Adsbrakecommand46* Adsbrakecommand46::set_epb_rampauxiliarycommand(
    bool epb_rampauxiliarycommand) {
  epb_rampauxiliarycommand_ = epb_rampauxiliarycommand;
  return this;
}

// config detail: {'description': '0x0:off;0x1:on', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'EPB_RampAuxiliaryCommand',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 25, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
void Adsbrakecommand46::set_p_epb_rampauxiliarycommand(
    uint8_t* data, bool epb_rampauxiliarycommand) {
  int x = epb_rampauxiliarycommand;

  Byte to_set(data + 3);
  to_set.set_value(x, 1, 1);
}

Adsbrakecommand46* Adsbrakecommand46::set_auto_drivercmd_alivecounter(
    int auto_drivercmd_alivecounter) {
  auto_drivercmd_alivecounter_ = auto_drivercmd_alivecounter;
  return this;
}

// config detail: {'name': 'AUTO_DriverCmd_AliveCounter', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Adsbrakecommand46::set_p_auto_drivercmd_alivecounter(
    uint8_t* data, int auto_drivercmd_alivecounter) {
  auto_drivercmd_alivecounter =
      ProtocolData::BoundedValue(0, 15, auto_drivercmd_alivecounter);
  int x = auto_drivercmd_alivecounter;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 4);
}

Adsbrakecommand46* Adsbrakecommand46::set_auto_drivercmd_checksum(
    int auto_drivercmd_checksum) {
  auto_drivercmd_checksum_ = auto_drivercmd_checksum;
  return this;
}

// config detail: {'name': 'AUTO_DriverCmd_CheckSum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Adsbrakecommand46::set_p_auto_drivercmd_checksum(
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
