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

class Pcepb203 : public ::apollo::drivers::canbus::ProtocolData<
                     ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Pcepb203();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': 'EPB request', 'enum': {0:
  // 'PC_EPBREQ_INVALID', 1: 'PC_EPBREQ_RELEASE', 2: 'PC_EPBREQ_APPLY'},
  // 'precision': 1.0, 'len': 2, 'name': 'PC_EpbReq', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Pcepb203* set_pc_epbreq(Pc_epb_203::Pc_epbreqType pc_epbreq);

  // config detail: {'description': 'EPB control enable', 'enum': {0:
  // 'PC_EPBENABLE_DISABLE', 1: 'PC_EPBENABLE_ENABLE'}, 'precision': 1.0, 'len':
  // 1, 'name': 'PC_EpbEnable', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Pcepb203* set_pc_epbenable(Pc_epb_203::Pc_epbenableType pc_epbenable);

 private:
  // config detail: {'description': 'EPB request', 'enum': {0:
  // 'PC_EPBREQ_INVALID', 1: 'PC_EPBREQ_RELEASE', 2: 'PC_EPBREQ_APPLY'},
  // 'precision': 1.0, 'len': 2, 'name': 'PC_EpbReq', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_pc_epbreq(uint8_t* data, Pc_epb_203::Pc_epbreqType pc_epbreq);

  // config detail: {'description': 'EPB control enable', 'enum': {0:
  // 'PC_EPBENABLE_DISABLE', 1: 'PC_EPBENABLE_ENABLE'}, 'precision': 1.0, 'len':
  // 1, 'name': 'PC_EpbEnable', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_pc_epbenable(uint8_t* data,
                          Pc_epb_203::Pc_epbenableType pc_epbenable);

 private:
  Pc_epb_203::Pc_epbreqType pc_epbreq_;
  Pc_epb_203::Pc_epbenableType pc_epbenable_;
};

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
