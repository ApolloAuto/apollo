/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/neolix_edu/protocol/ads_diagnosis_628.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

const int32_t Adsdiagnosis628::ID = 0x628;

// public
Adsdiagnosis628::Adsdiagnosis628() { Reset(); }

uint32_t Adsdiagnosis628::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Adsdiagnosis628::UpdateData(uint8_t* data) {
  set_p_faultrank(data, faultrank_);
  set_p_adas_fault_code(data, adas_fault_code_);
  set_p_adas_softwareversion(data, adas_softwareversion_);
  set_p_adas_hardwareversion(data, adas_hardwareversion_);
}

void Adsdiagnosis628::Reset() {
  // TODO(All) :  you should check this manually
  faultrank_ = 0;
  adas_fault_code_ = 0;
  adas_softwareversion_ = 0;
  adas_hardwareversion_ = 0;
}

Adsdiagnosis628* Adsdiagnosis628::set_faultrank(int faultrank) {
  faultrank_ = faultrank;
  return this;
}

// config detail: {'description': '0x0:Nomal;0x1:Level 1;0x2:Level 2;0x3:Level
// 3;0x4:Level 4;0x5:Level 5;0x6:Reserved;0x7:Reserved', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'name': 'FaultRank', 'is_signed_var': False,
// 'physical_range': '[0|5]', 'bit': 7, 'type': 'int', 'order': 'motorola',
// 'physical_unit': 'bit'}
void Adsdiagnosis628::set_p_faultrank(uint8_t* data, int faultrank) {
  faultrank = ProtocolData::BoundedValue(0, 5, faultrank);
  int x = faultrank;

  Byte to_set(data + 0);
  to_set.set_value(x, 4, 4);
}

Adsdiagnosis628* Adsdiagnosis628::set_adas_fault_code(int adas_fault_code) {
  adas_fault_code_ = adas_fault_code;
  return this;
}

// config detail: {'name': 'ADAS_Fault_Code', 'offset': 0.0, 'precision': 1.0,
// 'len': 24, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 3,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Adsdiagnosis628::set_p_adas_fault_code(uint8_t* data,
                                            int adas_fault_code) {
  adas_fault_code = ProtocolData::BoundedValue(0, 65535, adas_fault_code);
  int x = adas_fault_code;
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(data + 3);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = x & 0xFF;
  Byte to_set1(data + 2);
  to_set1.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set2(data + 1);
  to_set2.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xF;
  Byte to_set3(data + 0);
  to_set3.set_value(t, 0, 4);
}

Adsdiagnosis628* Adsdiagnosis628::set_adas_softwareversion(
    int adas_softwareversion) {
  adas_softwareversion_ = adas_softwareversion;
  return this;
}

// config detail: {'name': 'ADAS_SoftwareVersion', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit':
// 'bit'}
void Adsdiagnosis628::set_p_adas_softwareversion(uint8_t* data,
                                                 int adas_softwareversion) {
  adas_softwareversion =
      ProtocolData::BoundedValue(0, 255, adas_softwareversion);
  int x = adas_softwareversion;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 8);
}

Adsdiagnosis628* Adsdiagnosis628::set_adas_hardwareversion(
    int adas_hardwareversion) {
  adas_hardwareversion_ = adas_hardwareversion;
  return this;
}

// config detail: {'name': 'ADAS_HardwareVersion', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
// 'bit'}
void Adsdiagnosis628::set_p_adas_hardwareversion(uint8_t* data,
                                                 int adas_hardwareversion) {
  adas_hardwareversion =
      ProtocolData::BoundedValue(0, 255, adas_hardwareversion);
  int x = adas_hardwareversion;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
