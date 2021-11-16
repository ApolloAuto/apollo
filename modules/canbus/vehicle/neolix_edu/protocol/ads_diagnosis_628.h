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

class Adsdiagnosis628 : public ::apollo::drivers::canbus::ProtocolData<
                            ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Adsdiagnosis628();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': '0x0:Nomal;0x1:Level 1;0x2:Level 2;0x3:Level
  // 3;0x4:Level 4;0x5:Level 5;0x6:Reserved;0x7:Reserved', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'name': 'FaultRank', 'is_signed_var': False,
  // 'physical_range': '[0|5]', 'bit': 7, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'bit'}
  Adsdiagnosis628* set_faultrank(int faultrank);

  // config detail: {'name': 'ADAS_Fault_Code', 'offset': 0.0, 'precision': 1.0,
  // 'len': 24, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 3,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Adsdiagnosis628* set_adas_fault_code(int adas_fault_code);

  // config detail: {'name': 'ADAS_SoftwareVersion', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // 'bit'}
  Adsdiagnosis628* set_adas_softwareversion(int adas_softwareversion);

  // config detail: {'name': 'ADAS_HardwareVersion', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // 'bit'}
  Adsdiagnosis628* set_adas_hardwareversion(int adas_hardwareversion);

 private:
  // config detail: {'description': '0x0:Nomal;0x1:Level 1;0x2:Level 2;0x3:Level
  // 3;0x4:Level 4;0x5:Level 5;0x6:Reserved;0x7:Reserved', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'name': 'FaultRank', 'is_signed_var': False,
  // 'physical_range': '[0|5]', 'bit': 7, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'bit'}
  void set_p_faultrank(uint8_t* data, int faultrank);

  // config detail: {'name': 'ADAS_Fault_Code', 'offset': 0.0, 'precision': 1.0,
  // 'len': 24, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 3,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adas_fault_code(uint8_t* data, int adas_fault_code);

  // config detail: {'name': 'ADAS_SoftwareVersion', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // 'bit'}
  void set_p_adas_softwareversion(uint8_t* data, int adas_softwareversion);

  // config detail: {'name': 'ADAS_HardwareVersion', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // 'bit'}
  void set_p_adas_hardwareversion(uint8_t* data, int adas_hardwareversion);

 private:
  int faultrank_;
  int adas_fault_code_;
  int adas_softwareversion_;
  int adas_hardwareversion_;
};

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
