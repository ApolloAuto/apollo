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

#include "modules/canbus_vehicle/zhongyun/protocol/brake_control_a4.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

using ::apollo::drivers::canbus::Byte;

const int32_t Brakecontrola4::ID = 0xA4;

// public
Brakecontrola4::Brakecontrola4() { Reset(); }

uint32_t Brakecontrola4::GetPeriod() const {
  // TODO(ChaoM) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Brakecontrola4::UpdateData(uint8_t* data) {
  set_p_brake_torque(data, brake_torque_);
  set_p_brake_enable_control(data, brake_enable_control_);
}

void Brakecontrola4::Reset() {
  // TODO(ChaoM) :  you should check this manually
  brake_torque_ = 0.0;
  brake_enable_control_ = Brake_control_a4::BRAKE_ENABLE_CONTROL_BRAKE_MANUAL;
}

Brakecontrola4* Brakecontrola4::set_brake_torque(double brake_torque) {
  brake_torque_ = brake_torque;
  return this;
}

// config detail: {'name': 'brake_torque', 'offset': 0.0, 'precision': 0.05,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 8,
// 'type': 'double', 'order': 'intel', 'physical_unit': '%'}
void Brakecontrola4::set_p_brake_torque(uint8_t* data, double brake_torque) {
  brake_torque = ProtocolData::BoundedValue(0.0, 100.0, brake_torque);
  int x = static_cast<int>(brake_torque / 0.050000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 1);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 2);
  to_set1.set_value(t, 0, 8);
}

Brakecontrola4* Brakecontrola4::set_brake_enable_control(
    Brake_control_a4::Brake_enable_controlType brake_enable_control) {
  brake_enable_control_ = brake_enable_control;
  return this;
}

// config detail: {'name': 'Brake_Enable_control', 'enum':
// {0: 'BRAKE_ENABLE_CONTROL_BRAKE_MANUAL',
// 1: 'BRAKE_ENABLE_CONTROL_BRAKE_AUTO'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0,
// 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Brakecontrola4::set_p_brake_enable_control(
    uint8_t* data,
    Brake_control_a4::Brake_enable_controlType brake_enable_control) {
  int x = brake_enable_control;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
