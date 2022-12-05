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

#include "modules/canbus_vehicle/gem/protocol/global_cmd_69.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

const int32_t Globalcmd69::ID = 0x69;

// public
Globalcmd69::Globalcmd69() { Reset(); }

uint32_t Globalcmd69::GetPeriod() const {
  // TODO(QiL) :modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Globalcmd69::UpdateData(uint8_t* data) {
  set_p_pacmod_enable(data, pacmod_enable_);
  set_p_clear_override(data, clear_override_);
  set_p_ignore_override(data, ignore_override_);
}

void Globalcmd69::Reset() {
  // TODO(QiL) :you should check this manually
  pacmod_enable_ = Global_cmd_69::PACMOD_ENABLE_CONTROL_DISABLED;
  clear_override_ = Global_cmd_69::CLEAR_OVERRIDE_DON_T_CLEAR_ACTIVE_OVERRIDES;
  ignore_override_ = Global_cmd_69::IGNORE_OVERRIDE_DON_T_IGNORE_USER_OVERRIDES;
}

Globalcmd69* Globalcmd69::set_pacmod_enable(
    Global_cmd_69::Pacmod_enableType pacmod_enable) {
  pacmod_enable_ = pacmod_enable;
  return this;
}

// config detail: {'name': 'PACMOD_ENABLE', 'enum': {0:
// 'PACMOD_ENABLE_CONTROL_DISABLED', 1: 'PACMOD_ENABLE_CONTROL_ENABLED'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Globalcmd69::set_p_pacmod_enable(
    uint8_t* data, Global_cmd_69::Pacmod_enableType pacmod_enable) {
  uint8_t x = pacmod_enable;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 1);
}

Globalcmd69* Globalcmd69::set_clear_override(
    Global_cmd_69::Clear_overrideType clear_override) {
  clear_override_ = clear_override;
  return this;
}

// config detail: {'name': 'CLEAR_OVERRIDE', 'enum': {0:
// 'CLEAR_OVERRIDE_DON_T_CLEAR_ACTIVE_OVERRIDES', 1:
// 'CLEAR_OVERRIDE_CLEAR_ACTIVE_OVERRIDES'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Globalcmd69::set_p_clear_override(
    uint8_t* data, Global_cmd_69::Clear_overrideType clear_override) {
  uint8_t x = clear_override;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 1, 1);
}

Globalcmd69* Globalcmd69::set_ignore_override(
    Global_cmd_69::Ignore_overrideType ignore_override) {
  ignore_override_ = ignore_override;
  return this;
}

// config detail: {'name': 'IGNORE_OVERRIDE', 'enum': {0:
// 'IGNORE_OVERRIDE_DON_T_IGNORE_USER_OVERRIDES', 1:
// 'IGNORE_OVERRIDE_IGNORE_USER_OVERRIDES'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 2,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Globalcmd69::set_p_ignore_override(
    uint8_t* data, Global_cmd_69::Ignore_overrideType ignore_override) {
  uint8_t x = ignore_override;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 2, 1);
}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
