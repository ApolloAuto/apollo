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

#include "modules/canbus/vehicle/ge3/protocol/pc_bcs_202.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

const int32_t Pcbcs202::ID = 0x202;

// public
Pcbcs202::Pcbcs202() { Reset(); }

uint32_t Pcbcs202::GetPeriod() const {
  // modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Pcbcs202::UpdateData(uint8_t* data) {
  set_p_pc_brkpedreq(data, pc_brkpedreq_);
  set_p_pc_brkpedenable(data, pc_brkpedenable_);
}

void Pcbcs202::Reset() {
  // you should check this manually
  pc_brkpedreq_ = 0.0;
  pc_brkpedenable_ = Pc_bcs_202::PC_BRKPEDENABLE_DISABLE;
}

Pcbcs202* Pcbcs202::set_pc_brkpedreq(double pc_brkpedreq) {
  pc_brkpedreq_ = pc_brkpedreq;
  return this;
}

// config detail: {'description': 'Brake pedal request', 'offset': 0.0,
// 'precision': 0.1, 'len': 10, 'name': 'PC_BrkPedReq', 'is_signed_var': False,
// 'physical_range': '[0|100]', 'bit': 1, 'type': 'double', 'order': 'motorola',
// 'physical_unit': '%'}
void Pcbcs202::set_p_pc_brkpedreq(uint8_t* data, double pc_brkpedreq) {
  pc_brkpedreq = ProtocolData::BoundedValue(0.0, 100.0, pc_brkpedreq);
  int x = static_cast<int>(pc_brkpedreq / 0.100000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 1);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0x3;
  Byte to_set1(data + 0);
  to_set1.set_value(t, 0, 2);
}

Pcbcs202* Pcbcs202::set_pc_brkpedenable(
    Pc_bcs_202::Pc_brkpedenableType pc_brkpedenable) {
  pc_brkpedenable_ = pc_brkpedenable;
  return this;
}

// config detail: {'description': 'Brake pedal control enable', 'enum': {0:
// 'PC_BRKPEDENABLE_DISABLE', 1: 'PC_BRKPEDENABLE_ENABLE'}, 'precision': 1.0,
// 'len': 1, 'name': 'PC_BrkPedEnable', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Pcbcs202::set_p_pc_brkpedenable(
    uint8_t* data, Pc_bcs_202::Pc_brkpedenableType pc_brkpedenable) {
  int x = pc_brkpedenable;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 7, 1);
}

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
