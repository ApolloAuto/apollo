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

class Vcupowerstatus214 : public ::apollo::drivers::canbus::ProtocolData<
                              ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcupowerstatus214();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description':
  // '0x0:OFF;0x1:IG_ON;0x2:Power_ON;0x3:Auto_ON;0x4:Reserved;0x5:Reserved;0x6:Reserved;0x7:Reserved',
  // 'offset': 0.0, 'precision': 1.0, 'len': 3, 'name': 'VCU_PowerMode',
  // 'is_signed_var': False, 'physical_range': '[0|7]', 'bit': 11, 'type':
  // 'int', 'order': 'motorola', 'physical_unit': 'bit'}
  int vcu_powermode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Not
  // Available;0x1:Invalid;0x2:Valid;0x3:Reserved', 'offset': 0.0,
  // 'precision': 1.0, 'len': 2, 'name': 'VCU_PowerModeValid', 'is_signed_var':
  // False, 'physical_range': '[0|3]', 'bit': 13, 'type': 'int', 'order':
  // 'motorola', 'physical_unit': 'bit'}
  int vcu_powermodevalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:NotActivate;0x1:Activate', 'offset':
  // 0.0, 'precision': 1.0, 'len': 1, 'name': 'ReplaceBatteryStateIndication',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 14, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': 'bit'}
  bool replacebatterystateindication(const std::uint8_t* bytes,
                                     const int32_t length) const;

  // config detail: {'description': '0x0:Normal AEB;0x1:Forbidden', 'offset':
  // 0.0, 'precision': 1.0, 'len': 1, 'name': 'forbidden_aeb_signal',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 15, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': 'bit'}
  bool forbidden_aeb_signal(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'description': ';', 'offset': -400, 'precision': 0.02,
  // 'len': 16, 'name': 'chargedischargecurrent', 'is_signed_var': False,
  // 'physical_range': '[-400|910.68]', 'bit': 40, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': ''}
  float bcu_chargedischargecurrent(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'description': ';', 'offset': 0, 'precision': 0.01,
  // 'len': 16, 'name': 'batt_internalvoltage', 'is_signed_var': False,
  // 'physical_range': '[0|655.35]', 'bit': 54, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': ''}
  float bcu_batt_internalvoltage(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'name': 'VCU_DriverInfo_AliveCounter', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|15]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // 'bit'}
  int vcu_driverinfo_alivecounter(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'name': 'VCU_DriverInfo_CheckSum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // 'bit'}
  int vcu_driverinfo_checksum(const std::uint8_t* bytes,
                              const int32_t length) const;
};

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
