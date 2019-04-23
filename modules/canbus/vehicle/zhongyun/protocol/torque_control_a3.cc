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

#include "modules/canbus/vehicle/zhongyun/protocol/torque_control_a3.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

using ::apollo::drivers::canbus::Byte;

const int32_t Torquecontrola3::ID = 0xA3;

// public
Torquecontrola3::Torquecontrola3() { Reset(); }

uint32_t Torquecontrola3::GetPeriod() const {
  // TODO(ChaoM) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Torquecontrola3::UpdateData(uint8_t* data) {
  set_p_driven_torque(data, driven_torque_);
  set_p_driven_enable_control(data, driven_enable_control_);
}

void Torquecontrola3::Reset() {
  // TODO(ChaoM) :  you should check this manually
  driven_torque_ = 0.0;
  driven_enable_control_ =
      Torque_control_a3::DRIVEN_ENABLE_CONTROL_DRIVE_MANUAL;
}

Torquecontrola3* Torquecontrola3::set_driven_torque(double driven_torque) {
  driven_torque_ = driven_torque;
  return this;
}

// config detail: {'name': 'driven_torque', 'offset': 0.0, 'precision': 0.05,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 8,
// 'type': 'double', 'order': 'intel', 'physical_unit': '%'}
void Torquecontrola3::set_p_driven_torque(uint8_t* data, double driven_torque) {
  driven_torque = ProtocolData::BoundedValue(0.0, 100.0, driven_torque);
  int x = static_cast<int>(driven_torque / 0.050000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 1);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 2);
  to_set1.set_value(t, 0, 8);
}

Torquecontrola3* Torquecontrola3::set_driven_enable_control(
    Torque_control_a3::Driven_enable_controlType driven_enable_control) {
  driven_enable_control_ = driven_enable_control;
  return this;
}

// config detail: {'name': 'Driven_Enable_control', 'enum': {0:
// 'DRIVEN_ENABLE_CONTROL_DRIVE_MANUAL', 1: 'DRIVEN_ENABLE_CONTROL_DRIVE_AUTO'},
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'intel',
// 'physical_unit': ''}
void Torquecontrola3::set_p_driven_enable_control(
    uint8_t* data,
    Torque_control_a3::Driven_enable_controlType driven_enable_control) {
  int x = driven_enable_control;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
