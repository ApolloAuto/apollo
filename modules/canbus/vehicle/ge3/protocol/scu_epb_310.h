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

class Scuepb310 : public ::apollo::drivers::canbus::ProtocolData<
                      ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Scuepb310();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': 'EPS interrupt index', 'enum': {0:
  // 'EPB_INTIDX_NOINT', 1: 'EPB_INTIDX_OVERFLOW', 2: 'EPB_INTIDX_TIMEOUT'},
  // 'precision': 1.0, 'len': 3, 'name': 'EPB_IntIdx', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 10, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Scu_epb_310::Epb_intidxType epb_intidx(const std::uint8_t* bytes,
                                         const int32_t length) const;

  // config detail: {'description': 'EPB drive mode', 'enum': {0:
  // 'EPB_DRVMODE_INVALID', 1: 'EPB_DRVMODE_MANUAL', 2: 'EPB_DRVMODE_INTERRUPT',
  // 3: 'EPB_DRVMODE_AUTO'}, 'precision': 1.0, 'len': 2, 'name': 'EPB_DrvMode',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 6,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Scu_epb_310::Epb_drvmodeType epb_drvmode(const std::uint8_t* bytes,
                                           const int32_t length) const;

  // config detail: {'description': 'EPB system status', 'enum': {0:
  // 'EPB_SYSST_RELEASED', 1: 'EPB_SYSST_APPLIED', 2: 'EPB_SYSST_RELEASING', 3:
  // 'EPB_SYSST_FAULT', 4: 'EPB_SYSST_APPLYING', 5: 'EPB_SYSST_DISENGAGED'},
  // 'precision': 1.0, 'len': 3, 'name': 'EPB_SysSt', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 2, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Scu_epb_310::Epb_sysstType epb_sysst(const std::uint8_t* bytes,
                                       const int32_t length) const;

  // config detail: {'description': 'EPB fault status', 'enum': {0:
  // 'EPB_FAULTST_NORMAL', 1: 'EPB_FAULTST_FAULT'}, 'precision': 1.0, 'len': 1,
  // 'name': 'EPB_FaultSt', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_epb_310::Epb_faultstType epb_faultst(const std::uint8_t* bytes,
                                           const int32_t length) const;
};

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
