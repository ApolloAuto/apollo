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

class Scubcs1306 : public ::apollo::drivers::canbus::ProtocolData<
                       ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Scubcs1306();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': 'VDC active status', 'enum': {0:
  // 'BCS_AEBAVAILABLE_UNAVAILABLE', 1: 'BCS_AEBAVAILABLE_AVAILABLE'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCS_AEBAvailable', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 17, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Scu_bcs_1_306::Bcs_aebavailableType bcs_aebavailable(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'VDC active status', 'enum': {0:
  // 'BCS_CDDAVAILABLE_UNAVAILABLE', 1: 'BCS_CDDAVAILABLE_AVAILABLE'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCS_CDDAvailable', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Scu_bcs_1_306::Bcs_cddavailableType bcs_cddavailable(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Actual brake pedal position', 'offset':
  // 0.0, 'precision': 0.1, 'len': 10, 'name': 'BCS_BrkPedAct', 'is_signed_var':
  // False, 'physical_range': '[0|100]', 'bit': 15, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': '%'}
  double bcs_brkpedact(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'BCS interrupt index', 'enum': {0:
  // 'BCS_INTIDX_NOINT', 1: 'BCS_INTIDX_OVERFLOW', 2: 'BCS_INTIDX_TIMEOUT', 3:
  // 'BCS_INTIDX_ACCPEDINT', 4: 'BCS_INTIDX_BRKPEDINT', 5:
  // 'BCS_INTIDX_GEARINT'}, 'precision': 1.0, 'len': 3, 'name': 'BCS_IntIdx',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit':
  // 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Scu_bcs_1_306::Bcs_intidxType bcs_intidx(const std::uint8_t* bytes,
                                           const int32_t length) const;

  // config detail: {'description': 'VDC fault status', 'enum': {0:
  // 'BCS_VDCFAULTST_NORMAL', 1: 'BCS_VDCFAULTST_FAULT'}, 'precision': 1.0,
  // 'len': 1, 'name': 'BCS_VDCFaultSt', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_bcs_1_306::Bcs_vdcfaultstType bcs_vdcfaultst(const std::uint8_t* bytes,
                                                   const int32_t length) const;

  // config detail: {'description': 'VDC active status', 'enum': {0:
  // 'BCS_VDCACTIVEST_INACTIVE', 1: 'BCS_VDCACTIVEST_ACTIVE'}, 'precision': 1.0,
  // 'len': 1, 'name': 'BCS_VDCActiveSt', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_bcs_1_306::Bcs_vdcactivestType bcs_vdcactivest(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'ABS fault status', 'enum': {0:
  // 'BCS_ABSFAULTST_NORMAL', 1: 'BCS_ABSFAULTST_FAULT'}, 'precision': 1.0,
  // 'len': 1, 'name': 'BCS_ABSFaultSt', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_bcs_1_306::Bcs_absfaultstType bcs_absfaultst(const std::uint8_t* bytes,
                                                   const int32_t length) const;

  // config detail: {'description': 'ABS active status', 'enum': {0:
  // 'BCS_ABSACTIVEST_INACTIVE', 1: 'BCS_ABSACTIVEST_ACTIVE'}, 'precision': 1.0,
  // 'len': 1, 'name': 'BCS_ABSActiveSt', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 4, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_bcs_1_306::Bcs_absactivestType bcs_absactivest(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'BCS fault status', 'enum': {0:
  // 'BCS_FAULTST_NORMAL', 1: 'BCS_FAULTST_FAULT'}, 'precision': 1.0, 'len': 1,
  // 'name': 'BCS_FaultSt', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_bcs_1_306::Bcs_faultstType bcs_faultst(const std::uint8_t* bytes,
                                             const int32_t length) const;

  // config detail: {'description': 'BCS drive mode', 'enum': {0:
  // 'BCS_DRVMODE_INVALID', 1: 'BCS_DRVMODE_MANUAL', 2: 'BCS_DRVMODE_INTERRUPT',
  // 3: 'BCS_DRVMODE_AUTO'}, 'precision': 1.0, 'len': 2, 'name': 'BCS_DrvMode',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Scu_bcs_1_306::Bcs_drvmodeType bcs_drvmode(const std::uint8_t* bytes,
                                             const int32_t length) const;
};

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
