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

#include "modules/canbus/vehicle/hunter2/protocol/control_mode_setting_421.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

using ::apollo::drivers::canbus::Byte;

const int32_t Controlmodesetting421::ID = 0x421;

// public
Controlmodesetting421::Controlmodesetting421() { Reset(); }

uint32_t Controlmodesetting421::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Controlmodesetting421::UpdateData(uint8_t* data) {
  set_p_control_mode_setting(data, control_mode_setting_);
}

void Controlmodesetting421::Reset() {
  // TODO(All) :  you should check this manually
  control_mode_setting_ = Control_mode_setting_421::CONTROL_MODE_SETTING_STANDBY;
}

Controlmodesetting421* Controlmodesetting421::set_control_mode_setting(
    Control_mode_setting_421::Control_mode_settingType control_mode_setting) {
  control_mode_setting_ = control_mode_setting;
  return this;
 }

// config detail: {'bit': 7, 'enum': {0: 'CONTROL_MODE_SETTING_STANDBY', 1: 'CONTROL_MODE_SETTING_CAN_COMMAND_CONTROL'}, 'is_signed_var': False, 'len': 8, 'name': 'control_mode_setting', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Controlmodesetting421::set_p_control_mode_setting(uint8_t* data,
    Control_mode_setting_421::Control_mode_settingType control_mode_setting) {
  int x = control_mode_setting;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}

}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo
