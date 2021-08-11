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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

class Adsepscommand56 : public ::apollo::drivers::canbus::ProtocolData<
                            ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Adsepscommand56();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': '0x0:disable ;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'Drive_Enable', 'is_signed_var': False,
  // 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order': 'motorola',
  // 'physical_unit': ''}
  Adsepscommand56* set_drive_enable(bool drive_enable);

  // config detail: {'name': 'AUTO_Target_Angle', 'offset': -2048.0,
  // 'precision': 0.0625, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit':
  // ''}
  Adsepscommand56* set_auto_target_angle(double auto_target_angle);

  // config detail: {'name': 'AUTO_DriverCmd_AliveCounter', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  Adsepscommand56* set_auto_drivercmd_alivecounter(
      int auto_drivercmd_alivecounter);

  // config detail: {'name': 'AUTO_DriverCmd_CheckSum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  Adsepscommand56* set_auto_drivercmd_checksum(int auto_drivercmd_checksum);

 private:
  // config detail: {'description': '0x0:disable ;0x1:enable', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'Drive_Enable', 'is_signed_var': False,
  // 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_drive_enable(uint8_t* data, bool drive_enable);

  // config detail: {'name': 'AUTO_Target_Angle', 'offset': -2048.0,
  // 'precision': 0.0625, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit':
  // ''}
  void set_p_auto_target_angle(uint8_t* data, double auto_target_angle);

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
  bool drive_enable_;
  double auto_target_angle_;
  int auto_drivercmd_alivecounter_;
  int auto_drivercmd_checksum_;
};

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
