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

class Adsbrakecommand46 : public ::apollo::drivers::canbus::ProtocolData<
                              ::apollo::canbus::Neolix_edu> {
 public:
  static const int32_t ID;

  Adsbrakecommand46();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Neolix_edu* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': '0x0:disable ;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'Drive_Enable', 'is_signed_var': False,
  // 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order': 'motorola',
  // 'physical_unit': ''}
  Adsbrakecommand46* set_drive_enable(bool drive_enable);

  // config detail: {'name': 'AUTO_Brake_Command', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  Adsbrakecommand46* set_auto_brake_command(int auto_brake_command);

  // config detail: {'description': '0x0:Release ;0x1:Apply ', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'AUTO_Parking_Command',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 24, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  Adsbrakecommand46* set_auto_parking_command(bool auto_parking_command);

  // config detail: {'description': '0x0:off;0x1:on', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'EPB_RampAuxiliaryCommand',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 25, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  Adsbrakecommand46* set_epb_rampauxiliarycommand(
      bool epb_rampauxiliarycommand);

  // config detail: {'name': 'AUTO_DriverCmd_AliveCounter', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  Adsbrakecommand46* set_auto_drivercmd_alivecounter(
      int auto_drivercmd_alivecounter);

  // config detail: {'name': 'AUTO_DriverCmd_CheckSum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  Adsbrakecommand46* set_auto_drivercmd_checksum(int auto_drivercmd_checksum);

 private:
  // config detail: {'description': '0x0:disable ;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'Drive_Enable', 'is_signed_var': False,
  // 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_drive_enable(uint8_t* data, bool drive_enable);

  // config detail: {'name': 'AUTO_Brake_Command', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  void set_p_auto_brake_command(uint8_t* data, int auto_brake_command);

  // config detail: {'description': '0x0:Release ;0x1:Apply ', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'AUTO_Parking_Command',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 24, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  void set_p_auto_parking_command(uint8_t* data, bool auto_parking_command);

  // config detail: {'description': '0x0:off;0x1:on', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'EPB_RampAuxiliaryCommand',
  // 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 25, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  void set_p_epb_rampauxiliarycommand(uint8_t* data,
                                      bool epb_rampauxiliarycommand);

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

  bool drive_enable(const std::uint8_t* bytes, const int32_t length) const;

  bool auto_parking_command(const std::uint8_t* bytes, int32_t length) const;

  int32_t auto_brake_command(const std::uint8_t* bytes, int32_t length) const;

 private:
  bool drive_enable_;
  int auto_brake_command_;
  bool auto_parking_command_;
  bool epb_rampauxiliarycommand_;
  int auto_drivercmd_alivecounter_;
  int auto_drivercmd_checksum_;
};

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
