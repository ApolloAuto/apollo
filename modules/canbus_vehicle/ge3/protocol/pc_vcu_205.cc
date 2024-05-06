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

#include "modules/canbus_vehicle/ge3/protocol/pc_vcu_205.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

const int32_t Pcvcu205::ID = 0x205;

// public
Pcvcu205::Pcvcu205() { Reset(); }

uint32_t Pcvcu205::GetPeriod() const {
  // modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Pcvcu205::UpdateData(uint8_t* data) {
  set_p_pc_accpedreq(data, pc_accpedreq_);
  set_p_pc_accpedenable(data, pc_accpedenable_);
  set_p_pc_torqreq(data, pc_torqreq_);
  set_p_pc_torqenable(data, pc_torqenable_);
  set_p_pc_gearreq(data, pc_gearreq_);
  set_p_pc_gearenable(data, pc_gearenable_);
}

void Pcvcu205::Reset() {
  // you should check this manually
  pc_accpedreq_ = 0.0;
  pc_accpedenable_ = Pc_vcu_205::PC_ACCPEDENABLE_DISABLE;
  pc_torqreq_ = 0.0;
  pc_torqenable_ = Pc_vcu_205::PC_TORQENABLE_DISABLE;
  pc_gearreq_ = Pc_vcu_205::PC_GEARREQ_INVALID;
  pc_gearenable_ = Pc_vcu_205::PC_GEARENABLE_DISABLE;
}

Pcvcu205* Pcvcu205::set_pc_accpedreq(double pc_accpedreq) {
  pc_accpedreq_ = pc_accpedreq;
  return this;
}

// config detail: {'description': 'Acceleration pedal request', 'offset': 0.0,
// 'precision': 0.05, 'len': 12, 'name': 'PC_AccPedReq', 'is_signed_var': False,
// 'physical_range': '[0|100]', 'bit': 15, 'type': 'double', 'order':
// 'motorola', 'physical_unit': '%'}
void Pcvcu205::set_p_pc_accpedreq(uint8_t* data, double pc_accpedreq) {
  pc_accpedreq = ProtocolData::BoundedValue(0.0, 100.0, pc_accpedreq);
  int x = static_cast<int>(pc_accpedreq / 0.050000);
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}

Pcvcu205* Pcvcu205::set_pc_accpedenable(
    Pc_vcu_205::Pc_accpedenableType pc_accpedenable) {
  pc_accpedenable_ = pc_accpedenable;
  return this;
}

// config detail: {'description': 'Acceleration pedal control enable', 'enum':
// {0: 'PC_ACCPEDENABLE_DISABLE', 1: 'PC_ACCPEDENABLE_ENABLE'},
// 'precision': 1.0, 'len': 1, 'name': 'PC_AccPedEnable', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 6, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
void Pcvcu205::set_p_pc_accpedenable(
    uint8_t* data, Pc_vcu_205::Pc_accpedenableType pc_accpedenable) {
  int x = pc_accpedenable;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 6, 1);
}

Pcvcu205* Pcvcu205::set_pc_torqreq(double pc_torqreq) {
  pc_torqreq_ = pc_torqreq;
  return this;
}

// config detail: {'description': 'Torque request', 'offset': -3000.0,
// 'precision': 1.5, 'len': 12, 'name': 'PC_TorqReq', 'is_signed_var': False,
// 'physical_range': '[-3000|3000]', 'bit': 19, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'Nm'}
void Pcvcu205::set_p_pc_torqreq(uint8_t* data, double pc_torqreq) {
  pc_torqreq = ProtocolData::BoundedValue(-3000.0, 3000.0, pc_torqreq);
  int x = static_cast<int>((pc_torqreq - -3000.000000) / 1.500000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 3);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xF;
  Byte to_set1(data + 2);
  to_set1.set_value(t, 0, 4);
}

Pcvcu205* Pcvcu205::set_pc_torqenable(
    Pc_vcu_205::Pc_torqenableType pc_torqenable) {
  pc_torqenable_ = pc_torqenable;
  return this;
}

// config detail: {'description': 'Torque control enable', 'enum': {0:
// 'PC_TORQENABLE_DISABLE', 1: 'PC_TORQENABLE_ENABLE'}, 'precision': 1.0, 'len':
// 1, 'name': 'PC_TorqEnable', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Pcvcu205::set_p_pc_torqenable(
    uint8_t* data, Pc_vcu_205::Pc_torqenableType pc_torqenable) {
  int x = pc_torqenable;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 5, 1);
}

Pcvcu205* Pcvcu205::set_pc_gearreq(Pc_vcu_205::Pc_gearreqType pc_gearreq) {
  pc_gearreq_ = pc_gearreq;
  return this;
}

// config detail: {'description': 'Gear request', 'enum': {0:
// 'PC_GEARREQ_INVALID', 1: 'PC_GEARREQ_DRIVE', 2: 'PC_GEARREQ_NEUTRAL', 3:
// 'PC_GEARREQ_REVERSE', 4: 'PC_GEARREQ_PARK'}, 'precision': 1.0, 'len': 3,
// 'name': 'PC_GearReq', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|7]', 'bit': 2, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Pcvcu205::set_p_pc_gearreq(uint8_t* data,
                                Pc_vcu_205::Pc_gearreqType pc_gearreq) {
  int x = pc_gearreq;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 3);
}

Pcvcu205* Pcvcu205::set_pc_gearenable(
    Pc_vcu_205::Pc_gearenableType pc_gearenable) {
  pc_gearenable_ = pc_gearenable;
  return this;
}

// config detail: {'description': 'Gear control enable', 'enum': {0:
// 'PC_GEARENABLE_DISABLE', 1: 'PC_GEARENABLE_ENABLE'}, 'precision': 1.0, 'len':
// 1, 'name': 'PC_GearEnable', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Pcvcu205::set_p_pc_gearenable(
    uint8_t* data, Pc_vcu_205::Pc_gearenableType pc_gearenable) {
  int x = pc_gearenable;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 7, 1);
}

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
