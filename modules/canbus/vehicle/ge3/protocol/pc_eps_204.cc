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

#include "modules/canbus/vehicle/ge3/protocol/pc_eps_204.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

const int32_t Pceps204::ID = 0x204;

// public
Pceps204::Pceps204() { Reset(); }

uint32_t Pceps204::GetPeriod() const {
  // modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Pceps204::UpdateData(uint8_t* data) {
  set_p_pc_steerspdreq(data, pc_steerspdreq_);
  set_p_pc_steerenable(data, pc_steerenable_);
  set_p_pc_steerangreq(data, pc_steerangreq_);
}

void Pceps204::Reset() {
  // you should check this manually
  pc_steerspdreq_ = 0;
  pc_steerenable_ = Pc_eps_204::PC_STEERENABLE_DISABLE;
  pc_steerangreq_ = 0.0;
}

Pceps204* Pceps204::set_pc_steerspdreq(int pc_steerspdreq) {
  pc_steerspdreq_ = pc_steerspdreq;
  return this;
}

// config detail: {'description': 'Steer speed request', 'offset': 0.0,
// 'precision': 1.0, 'len': 16, 'name': 'PC_SteerSpdReq', 'is_signed_var':
// False, 'physical_range': '[0|500]', 'bit': 31, 'type': 'int', 'order':
// 'motorola', 'physical_unit': 'deg/s'}
void Pceps204::set_p_pc_steerspdreq(uint8_t* data, int pc_steerspdreq) {
  pc_steerspdreq = ProtocolData::BoundedValue(0, 500, pc_steerspdreq);
  int x = pc_steerspdreq;
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 4);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
}

Pceps204* Pceps204::set_pc_steerenable(
    Pc_eps_204::Pc_steerenableType pc_steerenable) {
  pc_steerenable_ = pc_steerenable;
  return this;
}

// config detail: {'description': 'Steer control enable', 'enum': {0:
// 'PC_STEERENABLE_DISABLE', 1: 'PC_STEERENABLE_ENABLE'}, 'precision': 1.0,
// 'len': 1, 'name': 'PC_SteerEnable', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Pceps204::set_p_pc_steerenable(
    uint8_t* data, Pc_eps_204::Pc_steerenableType pc_steerenable) {
  int x = pc_steerenable;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 7, 1);
}

Pceps204* Pceps204::set_pc_steerangreq(double pc_steerangreq) {
  pc_steerangreq_ = pc_steerangreq;
  return this;
}

// config detail: {'description': 'Steer angle request', 'offset': -500.0,
// 'precision': 0.1, 'len': 16, 'name': 'PC_SteerAngReq', 'is_signed_var':
// False, 'physical_range': '[-500|500]', 'bit': 15, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'deg'}
void Pceps204::set_p_pc_steerangreq(uint8_t* data, double pc_steerangreq) {
  pc_steerangreq = ProtocolData::BoundedValue(-500.0, 500.0, pc_steerangreq);
  int x = static_cast<int>((pc_steerangreq - -500.000000) / 0.100000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 2);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
