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

#include "modules/canbus_vehicle/ge3/protocol/pc_bcm_201.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

const int32_t Pcbcm201::ID = 0x201;

// public
Pcbcm201::Pcbcm201() { Reset(); }

uint32_t Pcbcm201::GetPeriod() const {
  // modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Pcbcm201::UpdateData(uint8_t* data) {
  set_p_pc_reverselampreq(data, pc_reverselampreq_);
  set_p_pc_lowbeamreq(data, pc_lowbeamreq_);
  set_p_pc_highbeamreq(data, pc_highbeamreq_);
  set_p_pc_rightturnlampreq(data, pc_rightturnlampreq_);
  set_p_pc_leftturnlampreq(data, pc_leftturnlampreq_);
  set_p_pc_hornreq(data, pc_hornreq_);
  set_p_pc_hazardlampreq(data, pc_hazardlampreq_);
}

void Pcbcm201::Reset() {
  // you should check this manually
  pc_reverselampreq_ = Pc_bcm_201::PC_REVERSELAMPREQ_NOREQ;
  pc_lowbeamreq_ = Pc_bcm_201::PC_LOWBEAMREQ_NOREQ;
  pc_highbeamreq_ = Pc_bcm_201::PC_HIGHBEAMREQ_NOREQ;
  pc_rightturnlampreq_ = Pc_bcm_201::PC_RIGHTTURNLAMPREQ_NOREQ;
  pc_leftturnlampreq_ = Pc_bcm_201::PC_LEFTTURNLAMPREQ_NOREQ;
  pc_hornreq_ = Pc_bcm_201::PC_HORNREQ_NOREQ;
  pc_hazardlampreq_ = Pc_bcm_201::PC_HAZARDLAMPREQ_NOREQ;
}

Pcbcm201* Pcbcm201::set_pc_reverselampreq(
    Pc_bcm_201::Pc_reverselampreqType pc_reverselampreq) {
  pc_reverselampreq_ = pc_reverselampreq;
  return this;
}

// config detail: {'description': 'Left turn lamp request', 'enum': {0:
// 'PC_REVERSELAMPREQ_NOREQ', 1: 'PC_REVERSELAMPREQ_REQ'}, 'precision': 1.0,
// 'len': 1, 'name': 'PC_ReverseLampReq', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Pcbcm201::set_p_pc_reverselampreq(
    uint8_t* data, Pc_bcm_201::Pc_reverselampreqType pc_reverselampreq) {
  int x = pc_reverselampreq;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 1, 1);
}

Pcbcm201* Pcbcm201::set_pc_lowbeamreq(
    Pc_bcm_201::Pc_lowbeamreqType pc_lowbeamreq) {
  pc_lowbeamreq_ = pc_lowbeamreq;
  return this;
}

// config detail: {'description': 'Left turn lamp request', 'enum': {0:
// 'PC_LOWBEAMREQ_NOREQ', 1: 'PC_LOWBEAMREQ_REQ'}, 'precision': 1.0, 'len': 1,
// 'name': 'PC_LowBeamReq', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Pcbcm201::set_p_pc_lowbeamreq(
    uint8_t* data, Pc_bcm_201::Pc_lowbeamreqType pc_lowbeamreq) {
  int x = pc_lowbeamreq;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 2, 1);
}

Pcbcm201* Pcbcm201::set_pc_highbeamreq(
    Pc_bcm_201::Pc_highbeamreqType pc_highbeamreq) {
  pc_highbeamreq_ = pc_highbeamreq;
  return this;
}

// config detail: {'description': 'Left turn lamp request', 'enum': {0:
// 'PC_HIGHBEAMREQ_NOREQ', 1: 'PC_HIGHBEAMREQ_REQ'}, 'precision': 1.0, 'len': 1,
// 'name': 'PC_HighBeamReq', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Pcbcm201::set_p_pc_highbeamreq(
    uint8_t* data, Pc_bcm_201::Pc_highbeamreqType pc_highbeamreq) {
  int x = pc_highbeamreq;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 3, 1);
}

Pcbcm201* Pcbcm201::set_pc_rightturnlampreq(
    Pc_bcm_201::Pc_rightturnlampreqType pc_rightturnlampreq) {
  pc_rightturnlampreq_ = pc_rightturnlampreq;
  return this;
}

// config detail: {'description': 'Right turn lamp request', 'enum': {0:
// 'PC_RIGHTTURNLAMPREQ_NOREQ', 1: 'PC_RIGHTTURNLAMPREQ_REQ'}, 'precision': 1.0,
// 'len': 1, 'name': 'PC_RightTurnLampReq', 'is_signed_var': False, 'offset':
// 0.0, 'physical_range': '[0|3]', 'bit': 4, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
void Pcbcm201::set_p_pc_rightturnlampreq(
    uint8_t* data, Pc_bcm_201::Pc_rightturnlampreqType pc_rightturnlampreq) {
  int x = pc_rightturnlampreq;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 4, 1);
}

Pcbcm201* Pcbcm201::set_pc_leftturnlampreq(
    Pc_bcm_201::Pc_leftturnlampreqType pc_leftturnlampreq) {
  pc_leftturnlampreq_ = pc_leftturnlampreq;
  return this;
}

// config detail: {'description': 'Left turn lamp request', 'enum': {0:
// 'PC_LEFTTURNLAMPREQ_NOREQ', 1: 'PC_LEFTTURNLAMPREQ_REQ'}, 'precision': 1.0,
// 'len': 1, 'name': 'PC_LeftTurnLampReq', 'is_signed_var': False, 'offset':
// 0.0, 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
void Pcbcm201::set_p_pc_leftturnlampreq(
    uint8_t* data, Pc_bcm_201::Pc_leftturnlampreqType pc_leftturnlampreq) {
  int x = pc_leftturnlampreq;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 5, 1);
}

Pcbcm201* Pcbcm201::set_pc_hornreq(Pc_bcm_201::Pc_hornreqType pc_hornreq) {
  pc_hornreq_ = pc_hornreq;
  return this;
}

// config detail: {'description': 'Horn request', 'enum': {0:
// 'PC_HORNREQ_NOREQ', 1: 'PC_HORNREQ_REQ'}, 'precision': 1.0, 'len': 1, 'name':
// 'PC_HornReq', 'is_signed_var': False, 'offset': 0.0, 'physical_range':
// '[0|1]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pcbcm201::set_p_pc_hornreq(uint8_t* data,
                                Pc_bcm_201::Pc_hornreqType pc_hornreq) {
  int x = pc_hornreq;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 6, 1);
}

Pcbcm201* Pcbcm201::set_pc_hazardlampreq(
    Pc_bcm_201::Pc_hazardlampreqType pc_hazardlampreq) {
  pc_hazardlampreq_ = pc_hazardlampreq;
  return this;
}

// config detail: {'description': 'Hazard lamp request', 'enum': {0:
// 'PC_HAZARDLAMPREQ_NOREQ', 1: 'PC_HAZARDLAMPREQ_REQ'}, 'precision': 1.0,
// 'len': 1, 'name': 'PC_HazardLampReq', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Pcbcm201::set_p_pc_hazardlampreq(
    uint8_t* data, Pc_bcm_201::Pc_hazardlampreqType pc_hazardlampreq) {
  int x = pc_hazardlampreq;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 7, 1);
}

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
