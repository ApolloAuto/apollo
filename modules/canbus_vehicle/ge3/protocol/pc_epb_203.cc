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

#include "modules/canbus/vehicle/ge3/protocol/pc_epb_203.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

const int32_t Pcepb203::ID = 0x203;

// public
Pcepb203::Pcepb203() { Reset(); }

uint32_t Pcepb203::GetPeriod() const {
  // modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Pcepb203::UpdateData(uint8_t* data) {
  set_p_pc_epbreq(data, pc_epbreq_);
  set_p_pc_epbenable(data, pc_epbenable_);
}

void Pcepb203::Reset() {
  // you should check this manually
  pc_epbreq_ = Pc_epb_203::PC_EPBREQ_INVALID;
  pc_epbenable_ = Pc_epb_203::PC_EPBENABLE_DISABLE;
}

Pcepb203* Pcepb203::set_pc_epbreq(Pc_epb_203::Pc_epbreqType pc_epbreq) {
  pc_epbreq_ = pc_epbreq;
  return this;
}

// config detail: {'description': 'EPB request', 'enum': {0:
// 'PC_EPBREQ_INVALID', 1: 'PC_EPBREQ_RELEASE', 2: 'PC_EPBREQ_APPLY'},
// 'precision': 1.0, 'len': 2, 'name': 'PC_EpbReq', 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
void Pcepb203::set_p_pc_epbreq(uint8_t* data,
                               Pc_epb_203::Pc_epbreqType pc_epbreq) {
  int x = pc_epbreq;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 2);
}

Pcepb203* Pcepb203::set_pc_epbenable(
    Pc_epb_203::Pc_epbenableType pc_epbenable) {
  pc_epbenable_ = pc_epbenable;
  return this;
}

// config detail: {'description': 'EPB control enable', 'enum': {0:
// 'PC_EPBENABLE_DISABLE', 1: 'PC_EPBENABLE_ENABLE'}, 'precision': 1.0, 'len':
// 1, 'name': 'PC_EpbEnable', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Pcepb203::set_p_pc_epbenable(uint8_t* data,
                                  Pc_epb_203::Pc_epbenableType pc_epbenable) {
  int x = pc_epbenable;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 7, 1);
}

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
