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

#include "modules/canbus_vehicle/transit/protocol/llc_diag_brakecontrol_721.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

const int32_t Llcdiagbrakecontrol721::ID = 0x721;

// public
Llcdiagbrakecontrol721::Llcdiagbrakecontrol721() { Reset(); }

uint32_t Llcdiagbrakecontrol721::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Llcdiagbrakecontrol721::UpdateData(uint8_t* data) {
  set_p_llc_dbg_brakepidcontribution_p(data, llc_dbg_brakepidcontribution_p_);
  set_p_llc_dbg_brakepidcontribution_i(data, llc_dbg_brakepidcontribution_i_);
  set_p_llc_dbg_brakepidcontribution_d(data, llc_dbg_brakepidcontribution_d_);
  set_p_llc_dbg_brakepid_output(data, llc_dbg_brakepid_output_);
  set_p_llc_dbg_brakepid_error(data, llc_dbg_brakepid_error_);
  set_p_llc_dbg_brakefeedforward(data, llc_dbg_brakefeedforward_);
}

void Llcdiagbrakecontrol721::Reset() {
  // TODO(All) :  you should check this manually
  llc_dbg_brakepidcontribution_p_ = 0.0;
  llc_dbg_brakepidcontribution_i_ = 0.0;
  llc_dbg_brakepidcontribution_d_ = 0.0;
  llc_dbg_brakepid_output_ = 0.0;
  llc_dbg_brakepid_error_ = 0;
  llc_dbg_brakefeedforward_ = 0.0;
}

Llcdiagbrakecontrol721*
Llcdiagbrakecontrol721::set_llc_dbg_brakepidcontribution_p(
    double llc_dbg_brakepidcontribution_p) {
  llc_dbg_brakepidcontribution_p_ = llc_dbg_brakepidcontribution_p;
  return this;
}

// config detail: {'description': 'Brake control loop P contribution', 'offset':
// 0.0, 'precision': 0.1, 'len': 10, 'name': 'LLC_DBG_BrakePidContribution_P',
// 'is_signed_var': True, 'physical_range': '[-51.2|51.1]', 'bit': 34, 'type':
// 'double', 'order': 'intel', 'physical_unit': 'mrev'}
void Llcdiagbrakecontrol721::set_p_llc_dbg_brakepidcontribution_p(
    uint8_t* data, double llc_dbg_brakepidcontribution_p) {
  llc_dbg_brakepidcontribution_p =
      ProtocolData::BoundedValue(-51.2, 51.1, llc_dbg_brakepidcontribution_p);
  int x = static_cast<int>(llc_dbg_brakepidcontribution_p / 0.100000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0x3F);
  Byte to_set0(data + 4);
  to_set0.set_value(t, 2, 6);
  x >>= 6;

  t = static_cast<uint8_t>(x & 0xF);
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0, 4);
}

Llcdiagbrakecontrol721*
Llcdiagbrakecontrol721::set_llc_dbg_brakepidcontribution_i(
    double llc_dbg_brakepidcontribution_i) {
  llc_dbg_brakepidcontribution_i_ = llc_dbg_brakepidcontribution_i;
  return this;
}

// config detail: {'description': 'Brake control loop I contribution', 'offset':
// 0.0, 'precision': 0.1, 'len': 10, 'name': 'LLC_DBG_BrakePidContribution_I',
// 'is_signed_var': True, 'physical_range': '[-51.2|51.1]', 'bit': 44, 'type':
// 'double', 'order': 'intel', 'physical_unit': 'mrev'}
void Llcdiagbrakecontrol721::set_p_llc_dbg_brakepidcontribution_i(
    uint8_t* data, double llc_dbg_brakepidcontribution_i) {
  llc_dbg_brakepidcontribution_i =
      ProtocolData::BoundedValue(-51.2, 51.1, llc_dbg_brakepidcontribution_i);
  int x = static_cast<int>(llc_dbg_brakepidcontribution_i / 0.100000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xF);
  Byte to_set0(data + 5);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = static_cast<uint8_t>(x & 0x3F);
  Byte to_set1(data + 6);
  to_set1.set_value(t, 0, 6);
}

Llcdiagbrakecontrol721*
Llcdiagbrakecontrol721::set_llc_dbg_brakepidcontribution_d(
    double llc_dbg_brakepidcontribution_d) {
  llc_dbg_brakepidcontribution_d_ = llc_dbg_brakepidcontribution_d;
  return this;
}

// config detail: {'description': 'Brake control loop D contribution', 'offset':
// 0.0, 'precision': 0.1, 'len': 10, 'name': 'LLC_DBG_BrakePidContribution_D',
// 'is_signed_var': True, 'physical_range': '[-51.2|51.1]', 'bit': 54, 'type':
// 'double', 'order': 'intel', 'physical_unit': 'mrev'}
void Llcdiagbrakecontrol721::set_p_llc_dbg_brakepidcontribution_d(
    uint8_t* data, double llc_dbg_brakepidcontribution_d) {
  llc_dbg_brakepidcontribution_d =
      ProtocolData::BoundedValue(-51.2, 51.1, llc_dbg_brakepidcontribution_d);
  int x = static_cast<int>(llc_dbg_brakepidcontribution_d / 0.100000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0x3);
  Byte to_set0(data + 6);
  to_set0.set_value(t, 6, 2);
  x >>= 2;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 7);
  to_set1.set_value(t, 0, 8);
}

Llcdiagbrakecontrol721* Llcdiagbrakecontrol721::set_llc_dbg_brakepid_output(
    double llc_dbg_brakepid_output) {
  llc_dbg_brakepid_output_ = llc_dbg_brakepid_output;
  return this;
}

// config detail: {'description': 'Brake control loop output', 'offset': 0.0,
// 'precision': 0.1, 'len': 10, 'name': 'LLC_DBG_BrakePID_Output',
// 'is_signed_var': True, 'physical_range': '[-51.2|51.1]', 'bit': 12, 'type':
// 'double', 'order': 'intel', 'physical_unit': 'mrev'}
void Llcdiagbrakecontrol721::set_p_llc_dbg_brakepid_output(
    uint8_t* data, double llc_dbg_brakepid_output) {
  llc_dbg_brakepid_output =
      ProtocolData::BoundedValue(-51.2, 51.1, llc_dbg_brakepid_output);
  int x = static_cast<int>(llc_dbg_brakepid_output / 0.100000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xF);
  Byte to_set0(data + 1);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = static_cast<uint8_t>(x & 0x3F);
  Byte to_set1(data + 2);
  to_set1.set_value(t, 0, 6);
}

Llcdiagbrakecontrol721* Llcdiagbrakecontrol721::set_llc_dbg_brakepid_error(
    int llc_dbg_brakepid_error) {
  llc_dbg_brakepid_error_ = llc_dbg_brakepid_error;
  return this;
}

// config detail: {'description': 'Brake control loop error', 'offset': 0.0,
// 'precision': 1.0, 'len': 12, 'name': 'LLC_DBG_BrakePID_Error',
// 'is_signed_var': True, 'physical_range': '[-2048|2047]', 'bit': 0, 'type':
// 'int', 'order': 'intel', 'physical_unit': 'psi'}
void Llcdiagbrakecontrol721::set_p_llc_dbg_brakepid_error(
    uint8_t* data, int llc_dbg_brakepid_error) {
  llc_dbg_brakepid_error =
      ProtocolData::BoundedValue(-2048, 2047, llc_dbg_brakepid_error);
  int x = llc_dbg_brakepid_error;
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 0);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xF);
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 4);
}

Llcdiagbrakecontrol721* Llcdiagbrakecontrol721::set_llc_dbg_brakefeedforward(
    double llc_dbg_brakefeedforward) {
  llc_dbg_brakefeedforward_ = llc_dbg_brakefeedforward;
  return this;
}

// config detail: {'description': 'Brake control feedforward contribution',
// 'offset': 0.0, 'precision': 0.5, 'len': 12, 'name':
// 'LLC_DBG_BrakeFeedforward', 'is_signed_var': True, 'physical_range':
// '[-1024|1023.5]', 'bit': 22, 'type': 'double', 'order': 'intel',
// 'physical_unit': 'mrev'}
void Llcdiagbrakecontrol721::set_p_llc_dbg_brakefeedforward(
    uint8_t* data, double llc_dbg_brakefeedforward) {
  llc_dbg_brakefeedforward =
      ProtocolData::BoundedValue(-1024.0, 1023.5, llc_dbg_brakefeedforward);
  int x = static_cast<int>(llc_dbg_brakefeedforward / 0.500000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0x3);
  Byte to_set0(data + 2);
  to_set0.set_value(t, 6, 2);
  x >>= 2;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0x3);
  Byte to_set2(data + 4);
  to_set2.set_value(t, 0, 2);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
