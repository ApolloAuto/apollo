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

#include "modules/canbus/vehicle/lexus/protocol/cruise_control_buttons_cmd_108.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

const int32_t Cruisecontrolbuttonscmd108::ID = 0x108;

// public
Cruisecontrolbuttonscmd108::Cruisecontrolbuttonscmd108() { Reset(); }

uint32_t Cruisecontrolbuttonscmd108::GetPeriod() const {
  // TODO(QiL) modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Cruisecontrolbuttonscmd108::UpdateData(uint8_t* data) {
  set_p_cruise_control_button(data, cruise_control_button_);
  set_p_ignore_overrides(data, ignore_overrides_);
  set_p_clear_override(data, clear_override_);
  set_p_enable(data, enable_);
  set_p_clear_faults(data, clear_faults_);
}

void Cruisecontrolbuttonscmd108::Reset() {
  // TODO(QiL) you should check this manually
  cruise_control_button_ =
      Cruise_control_buttons_cmd_108::CRUISE_CONTROL_BUTTON_CRUISE_CONTROL_NONE;
  ignore_overrides_ = false;
  clear_override_ = false;
  enable_ = false;
  clear_faults_ = false;
}

Cruisecontrolbuttonscmd108*
Cruisecontrolbuttonscmd108::set_cruise_control_button(
    Cruise_control_buttons_cmd_108::Cruise_control_buttonType
        cruise_control_button) {
  cruise_control_button_ = cruise_control_button;
  return this;
}

// config detail: {'name': 'CRUISE_CONTROL_BUTTON', 'enum': {0:
// 'CRUISE_CONTROL_BUTTON_CRUISE_CONTROL_NONE', 1:
// 'CRUISE_CONTROL_BUTTON_CRUISE_CONTROL_CNCL', 2:
// 'CRUISE_CONTROL_BUTTON_CRUISE_CONTROL_ACC_FURTHER', 3:
// 'CRUISE_CONTROL_BUTTON_CRUISE_CONTROL_ACC_CLOSER', 4:
// 'CRUISE_CONTROL_BUTTON_CRUISE_CONTROL_SET_DEC', 5:
// 'CRUISE_CONTROL_BUTTON_CRUISE_CONTROL_RES_INC', 6:
// 'CRUISE_CONTROL_BUTTON_CRUISE_CONTROL_ON_OFF'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit':
// 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Cruisecontrolbuttonscmd108::set_p_cruise_control_button(
    uint8_t* data, Cruise_control_buttons_cmd_108::Cruise_control_buttonType
                       cruise_control_button) {
  uint8_t x = cruise_control_button;

  Byte to_set(data + 1);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

Cruisecontrolbuttonscmd108* Cruisecontrolbuttonscmd108::set_ignore_overrides(
    bool ignore_overrides) {
  ignore_overrides_ = ignore_overrides;
  return this;
}

// config detail: {'name': 'IGNORE_OVERRIDES', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
void Cruisecontrolbuttonscmd108::set_p_ignore_overrides(uint8_t* data,
                                                        bool ignore_overrides) {
  uint8_t x = ignore_overrides;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 1, 1);
}

Cruisecontrolbuttonscmd108* Cruisecontrolbuttonscmd108::set_clear_override(
    bool clear_override) {
  clear_override_ = clear_override;
  return this;
}

// config detail: {'name': 'CLEAR_OVERRIDE', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
void Cruisecontrolbuttonscmd108::set_p_clear_override(uint8_t* data,
                                                      bool clear_override) {
  uint8_t x = clear_override;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 2, 1);
}

Cruisecontrolbuttonscmd108* Cruisecontrolbuttonscmd108::set_enable(
    bool enable) {
  enable_ = enable;
  return this;
}

// config detail: {'name': 'ENABLE', 'offset': 0.0, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
void Cruisecontrolbuttonscmd108::set_p_enable(uint8_t* data, bool enable) {
  uint8_t x = enable;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 1);
}

Cruisecontrolbuttonscmd108* Cruisecontrolbuttonscmd108::set_clear_faults(
    bool clear_faults) {
  clear_faults_ = clear_faults;
  return this;
}

// config detail: {'name': 'CLEAR_FAULTS', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 3,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
void Cruisecontrolbuttonscmd108::set_p_clear_faults(uint8_t* data,
                                                    bool clear_faults) {
  uint8_t x = clear_faults;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 3, 1);
}

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
