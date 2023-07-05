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

class Aebsystemstate11 : public ::apollo::drivers::canbus::ProtocolData<
                             ::apollo::canbus::Neolix_edu> {
 public:
  static const int32_t ID;
  Aebsystemstate11();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Neolix_edu* chassis) const override;

 private:
  // config detail: {'description': '0x00:read only;0x01:brake enable',
  // 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name': 'AEB_State',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1, 'type': 'int',
  // 'order': 'motorola', 'physical_unit': ''}
  int aeb_state(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x00:off;0x01:on', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'AEB_BrakeState', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 2, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': ''}
  bool aeb_brakestate(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Nomal;0x1:Level 1;0x2:Level 2;0x3:Level
  // 3;0x4:Level 4;0x5:Level 5;0x6:Reserved;0x7:Reserved', 'offset': 0.0,
  // 'precision': 1.0, 'len': 3, 'name': 'FaultRank', 'is_signed_var': False,
  // 'physical_range': '[0|5]', 'bit': 10, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': ''}
  int faultrank(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CurrentTemperature', 'offset': -40.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|120]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int currenttemperature(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset':
  // 0.0, 'precision': 1.0, 'len': 1, 'name': 'PAS_F1_Stop', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 24, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': ''}
  bool pas_f1_stop(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset':
  // 0.0, 'precision': 1.0, 'len': 1, 'name': 'PAS_F2_Stop', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 25, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': ''}
  bool pas_f2_stop(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset':
  // 0.0, 'precision': 1.0, 'len': 1, 'name': 'PAS_F3_Stop', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 26, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': ''}
  bool pas_f3_stop(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset':
  // 0.0, 'precision': 1.0, 'len': 1, 'name': 'PAS_F4_Stop', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 27, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': ''}
  bool pas_f4_stop(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset':
  // 0.0, 'precision': 1.0, 'len': 1, 'name': 'PAS_B1_Stop', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 28, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': ''}
  bool pas_b1_stop(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset':
  // 0.0, 'precision': 1.0, 'len': 1, 'name': 'PAS_B2_Stop', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 29, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': ''}
  bool pas_b2_stop(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset':
  // 0.0, 'precision': 1.0, 'len': 1, 'name': 'PAS_B3_Stop', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 30, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': ''}
  bool pas_b3_stop(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Normal;0x1:ActivateBrake', 'offset':
  // 0.0, 'precision': 1.0, 'len': 1, 'name': 'PAS_B4_Stop', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 31, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': ''}
  bool pas_b4_stop(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'AEB_LiveCounter_Rear', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int aeb_livecounter_rear(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'name': 'AEB_Cheksum', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': 'bit'}
  int aeb_cheksum(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
