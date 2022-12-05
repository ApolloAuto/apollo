/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/transit/protocol/llc_diag_steeringcontrol_722.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

const int32_t Llcdiagsteeringcontrol722::ID = 0x722;

// public
Llcdiagsteeringcontrol722::Llcdiagsteeringcontrol722() { Reset(); }

uint32_t Llcdiagsteeringcontrol722::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Llcdiagsteeringcontrol722::UpdateData(uint8_t* data) {
  set_p_llc_dbg_steeringsensorposition(data, llc_dbg_steeringsensorposition_);
  set_p_llc_dbg_steeringrackinputtorque(data, llc_dbg_steeringrackinputtorque_);
  set_p_llc_dbg_steeringmotorposition(data, llc_dbg_steeringmotorposition_);
}

void Llcdiagsteeringcontrol722::Reset() {
  // TODO(All) :  you should check this manually
  llc_dbg_steeringsensorposition_ = 0.0;
  llc_dbg_steeringrackinputtorque_ = 0;
  llc_dbg_steeringmotorposition_ = 0.0;
}

Llcdiagsteeringcontrol722*
Llcdiagsteeringcontrol722::set_llc_dbg_steeringsensorposition(
    double llc_dbg_steeringsensorposition) {
  llc_dbg_steeringsensorposition_ = llc_dbg_steeringsensorposition;
  return this;
}

// config detail: {'description': 'Brake control feedforward contribution',
// 'offset': 0.0, 'precision': 0.0002, 'len': 16, 'name':
// 'LLC_DBG_SteeringSensorPosition', 'is_signed_var': True, 'physical_range':
// '[-6.5536|6.5534]', 'bit': 40, 'type': 'double', 'order': 'intel',
// 'physical_unit': 'rev'}
void Llcdiagsteeringcontrol722::set_p_llc_dbg_steeringsensorposition(
    uint8_t* data, double llc_dbg_steeringsensorposition) {
  llc_dbg_steeringsensorposition = ProtocolData::BoundedValue(
      -6.5536, 6.5534, llc_dbg_steeringsensorposition);
  int x = static_cast<int>(llc_dbg_steeringsensorposition / 0.000200);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 5);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 6);
  to_set1.set_value(t, 0, 8);
}

Llcdiagsteeringcontrol722*
Llcdiagsteeringcontrol722::set_llc_dbg_steeringrackinputtorque(
    int llc_dbg_steeringrackinputtorque) {
  llc_dbg_steeringrackinputtorque_ = llc_dbg_steeringrackinputtorque;
  return this;
}

// config detail: {'description': 'Brake control feedforward contribution',
// 'offset': 0.0, 'precision': 1.0, 'len': 16, 'name':
// 'LLC_DBG_SteeringRackInputTorque', 'is_signed_var': True, 'physical_range':
// '[-32768|32767]', 'bit': 24, 'type': 'int', 'order': 'intel',
// 'physical_unit': 'counts'}
void Llcdiagsteeringcontrol722::set_p_llc_dbg_steeringrackinputtorque(
    uint8_t* data, int llc_dbg_steeringrackinputtorque) {
  llc_dbg_steeringrackinputtorque = ProtocolData::BoundedValue(
      -32768, 32767, llc_dbg_steeringrackinputtorque);
  int x = llc_dbg_steeringrackinputtorque;
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 3);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 4);
  to_set1.set_value(t, 0, 8);
}

Llcdiagsteeringcontrol722*
Llcdiagsteeringcontrol722::set_llc_dbg_steeringmotorposition(
    double llc_dbg_steeringmotorposition) {
  llc_dbg_steeringmotorposition_ = llc_dbg_steeringmotorposition;
  return this;
}

// config detail: {'description': 'Brake control feedforward contribution',
// 'offset': 0.0, 'precision': 1e-05, 'len': 24, 'name':
// 'LLC_DBG_SteeringMotorPosition', 'is_signed_var': True, 'physical_range':
// '[-83.88608|83.88607]', 'bit': 0, 'type': 'double', 'order': 'intel',
// 'physical_unit': 'rev'}
void Llcdiagsteeringcontrol722::set_p_llc_dbg_steeringmotorposition(
    uint8_t* data, double llc_dbg_steeringmotorposition) {
  llc_dbg_steeringmotorposition = ProtocolData::BoundedValue(
      -83.88608, 83.88607, llc_dbg_steeringmotorposition);
  int x = static_cast<int>(llc_dbg_steeringmotorposition / 0.000010);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 0);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set2(data + 2);
  to_set2.set_value(t, 0, 8);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
