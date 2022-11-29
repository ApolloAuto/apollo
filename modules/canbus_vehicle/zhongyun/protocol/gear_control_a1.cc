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

#include "modules/canbus_vehicle/zhongyun/protocol/gear_control_a1.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

using ::apollo::drivers::canbus::Byte;

const int32_t Gearcontrola1::ID = 0xA1;

// public
Gearcontrola1::Gearcontrola1() { Reset(); }

uint32_t Gearcontrola1::GetPeriod() const {
  // TODO(ChaoM) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Gearcontrola1::UpdateData(uint8_t* data) {
  set_p_gear_state_target(data, gear_state_target_);
  set_p_gear_enable_control(data, gear_enable_control_);
}

void Gearcontrola1::Reset() {
  // TODO(ChaoM) :  you should check this manually
  gear_state_target_ = Gear_control_a1::GEAR_STATE_TARGET_P;
  gear_enable_control_ =
      Gear_control_a1::GEAR_ENABLE_CONTROL_GEAR_MANUALCONTROL;
}

Gearcontrola1* Gearcontrola1::set_gear_state_target(
    Gear_control_a1::Gear_state_targetType gear_state_target) {
  gear_state_target_ = gear_state_target;
  return this;
}

// config detail: {'name': 'Gear_state_target', 'enum': {1:
// 'GEAR_STATE_TARGET_P', 2: 'GEAR_STATE_TARGET_N', 3: 'GEAR_STATE_TARGET_D', 4:
// 'GEAR_STATE_TARGET_R', 5: 'GEAR_STATE_TARGET_INVALID'}, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[1|5]',
// 'bit': 8, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Gearcontrola1::set_p_gear_state_target(
    uint8_t* data, Gear_control_a1::Gear_state_targetType gear_state_target) {
  int x = gear_state_target;

  Byte to_set(data + 1);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

Gearcontrola1* Gearcontrola1::set_gear_enable_control(
    Gear_control_a1::Gear_enable_controlType gear_enable_control) {
  gear_enable_control_ = gear_enable_control;
  return this;
}

// config detail: {'name': 'Gear_Enable_control', 'enum': {0:
// 'GEAR_ENABLE_CONTROL_GEAR_MANUALCONTROL', 1:
// 'GEAR_ENABLE_CONTROL_GEAR_AUTOCONTROL'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0,
// 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Gearcontrola1::set_p_gear_enable_control(
    uint8_t* data,
    Gear_control_a1::Gear_enable_controlType gear_enable_control) {
  int x = gear_enable_control;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
