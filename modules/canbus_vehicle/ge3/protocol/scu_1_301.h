/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/ge3/proto/ge3.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ge3 {

class Scu1301 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::Ge3> {
 public:
  static const int32_t ID;
  Scu1301();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ge3* chassis) const override;

 private:
  // config detail: {'description': 'VIN string character 16', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN16', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 15, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin16(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Brake pedal position', 'enum': {0:
  // 'SCU_STOPBUTST_UNPRESSED', 1: 'SCU_STOPBUTST_PRESSED'}, 'precision': 1.0,
  // 'len': 1, 'name': 'SCU_StopButSt', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_1_301::Scu_stopbutstType scu_stopbutst(const std::uint8_t* bytes,
                                             const int32_t length) const;

  // config detail: {'description': 'SCU drive mode', 'enum': {0:
  // 'SCU_DRVMODE_INVALID', 1: 'SCU_DRVMODE_MANUAL', 2: 'SCU_DRVMODE_INTERRUPT',
  // 3: 'SCU_DRVMODE_AUTO'}, 'precision': 1.0, 'len': 2, 'name': 'SCU_DrvMode',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 3,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Scu_1_301::Scu_drvmodeType scu_drvmode(const std::uint8_t* bytes,
                                         const int32_t length) const;

  // config detail: {'description': 'SCU fault status', 'enum': {0:
  // 'SCU_FAULTST_NORMAL', 1: 'SCU_FAULTST_FAULT'}, 'precision': 1.0, 'len': 4,
  // 'name': 'SCU_FaultSt', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|15]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_1_301::Scu_faultstType scu_faultst(const std::uint8_t* bytes,
                                         const int32_t length) const;
};

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
