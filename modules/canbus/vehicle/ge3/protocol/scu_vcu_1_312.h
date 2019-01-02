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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ge3 {

class Scuvcu1312 : public ::apollo::drivers::canbus::ProtocolData<
                       ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Scuvcu1312();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': 'Gear fault status', 'enum': {0:
  // 'VCU_ELCSYSFAULT_NORMAL', 1: 'VCU_ELCSYSFAULT_FAULT'}, 'precision': 1.0,
  // 'len': 1, 'name': 'VCU_ElcSysFault', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 49, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_vcu_1_312::Vcu_elcsysfaultType vcu_elcsysfault(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Brake pedal position', 'enum': {0:
  // 'VCU_BRKPEDST_UNPRESSED', 1: 'VCU_BRKPEDST_PRESSED'}, 'precision': 1.0,
  // 'len': 1, 'name': 'VCU_BrkPedSt', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 48, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_vcu_1_312::Vcu_brkpedstType vcu_brkpedst(const std::uint8_t* bytes,
                                               const int32_t length) const;

  // config detail: {'description': 'VCU interrupt index', 'enum': {0:
  // 'VCU_INTIDX_NOINT', 1: 'VCU_INTIDX_OVERFLOW', 2: 'VCU_INTIDX_TIMEOUT', 3:
  // 'VCU_INTIDX_ACCPEDINT', 4: 'VCU_INTIDX_BRKPEDINT', 5:
  // 'VCU_INTIDX_GEARINT'}, 'precision': 1.0, 'len': 3, 'name': 'VCU_IntIdx',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit':
  // 58, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Scu_vcu_1_312::Vcu_intidxType vcu_intidx(const std::uint8_t* bytes,
                                           const int32_t length) const;

  // config detail: {'description': 'Gear interrupt index', 'enum': {0:
  // 'VCU_GEARINTIDX_NOINT', 1: 'VCU_GEARINTIDX_OVERFLOW', 2:
  // 'VCU_GEARINTIDX_TIMEOUT'}, 'precision': 1.0, 'len': 3, 'name':
  // 'VCU_GearIntIdx', 'is_signed_var': False, 'offset': 0.0, 'physical_range':
  // '[0|7]', 'bit': 61, 'type': 'enum', 'order': 'motorola', 'physical_unit':
  // ''}
  Scu_vcu_1_312::Vcu_gearintidxType vcu_gearintidx(const std::uint8_t* bytes,
                                                   const int32_t length) const;

  // config detail: {'description': 'VCU Gear drive mode', 'enum': {0:
  // 'VCU_GEARDRVMODE_INVALID', 1: 'VCU_GEARDRVMODE_MANUAL', 2:
  // 'VCU_GEARDRVMODE_INTERRUPT', 3: 'VCU_GEARDRVMODE_AUTO'}, 'precision': 1.0,
  // 'len': 2, 'name': 'VCU_GearDrvMode', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|3]', 'bit': 63, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_vcu_1_312::Vcu_geardrvmodeType vcu_geardrvmode(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Actual acceleration pedal position',
  // 'offset': 0.0, 'precision': 0.05, 'len': 12, 'name': 'VCU_AccPedAct',
  // 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 47, 'type':
  // 'double', 'order': 'motorola', 'physical_unit': '%'}
  double vcu_accpedact(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Brake pedal position', 'offset': 0.0,
  // 'precision': 0.392, 'len': 8, 'name': 'VCU_BrkPedPst', 'is_signed_var':
  // False, 'physical_range': '[0|99.96]', 'bit': 39, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': '%'}
  double vcu_brkpedpst(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Veh range', 'offset': 0.0,
  // 'precision': 1.0, 'len': 10, 'name': 'VCU_VehRng', 'is_signed_var': False,
  // 'physical_range': '[0|1000]', 'bit': 9, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'km'}
  int vcu_vehrng(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Actual acceleration pedal position',
  // 'offset': 0.0, 'precision': 0.392, 'len': 8, 'name': 'VCU_AccPedPst',
  // 'is_signed_var': False, 'physical_range': '[0|99.96]', 'bit': 31, 'type':
  // 'double', 'order': 'motorola', 'physical_unit': '%'}
  double vcu_accpedpst(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'HEV system ready status', 'enum': {0:
  // 'VCU_VEHRDYST_NOTREADY', 1: 'VCU_VEHRDYST_READY'}, 'precision': 1.0, 'len':
  // 1, 'name': 'VCU_VehRdySt', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_vcu_1_312::Vcu_vehrdystType vcu_vehrdyst(const std::uint8_t* bytes,
                                               const int32_t length) const;

  // config detail: {'description': 'VCU fault status', 'enum': {0:
  // 'VCU_FAULTST_NORMAL', 1: 'VCU_FAULTST_DERATE', 2: 'VCU_FAULTST_RSV1', 3:
  // 'VCU_FAULTST_RSV2', 4: 'VCU_FAULTST_RSV3', 5: 'VCU_FAULTST_FAULT'},
  // 'precision': 1.0, 'len': 4, 'name': 'VCU_FaultSt', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 5, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Scu_vcu_1_312::Vcu_faultstType vcu_faultst(const std::uint8_t* bytes,
                                             const int32_t length) const;

  // config detail: {'description': 'VCU drive mode', 'enum': {0:
  // 'VCU_DRVMODE_INVALID', 1: 'VCU_DRVMODE_MANUAL', 2: 'VCU_DRVMODE_INTERRUPT',
  // 3: 'VCU_DRVMODE_AUTO'}, 'precision': 1.0, 'len': 2, 'name': 'VCU_DrvMode',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Scu_vcu_1_312::Vcu_drvmodeType vcu_drvmode(const std::uint8_t* bytes,
                                             const int32_t length) const;

  // config detail: {'description': 'Gear lever position', 'enum': {0:
  // 'VCU_GEARPST_INVALID', 1: 'VCU_GEARPST_DRIVE', 2: 'VCU_GEARPST_NEUTRAL', 3:
  // 'VCU_GEARPST_REVERSE', 4: 'VCU_GEARPST_PARK'}, 'precision': 1.0, 'len': 3,
  // 'name': 'VCU_GearPst', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|7]', 'bit': 12, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_vcu_1_312::Vcu_gearpstType vcu_gearpst(const std::uint8_t* bytes,
                                             const int32_t length) const;

  // config detail: {'description': 'Gear fault status', 'enum': {0:
  // 'VCU_GEARFAULTST_NORMAL', 1: 'VCU_GEARFAULTST_FAULT'}, 'precision': 1.0,
  // 'len': 1, 'name': 'VCU_GearFaultSt', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 1, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_vcu_1_312::Vcu_gearfaultstType vcu_gearfaultst(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Actual gear', 'enum': {0:
  // 'VCU_GEARACT_INVALID', 1: 'VCU_GEARACT_DRIVE', 2: 'VCU_GEARACT_NEUTRAL', 3:
  // 'VCU_GEARACT_REVERSE', 4: 'VCU_GEARACT_PARK'}, 'precision': 1.0, 'len': 3,
  // 'name': 'VCU_GearAct', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|7]', 'bit': 15, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_vcu_1_312::Vcu_gearactType vcu_gearact(const std::uint8_t* bytes,
                                             const int32_t length) const;
};

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
