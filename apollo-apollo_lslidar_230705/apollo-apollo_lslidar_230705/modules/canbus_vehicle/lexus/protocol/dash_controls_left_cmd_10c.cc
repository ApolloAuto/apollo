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

#include "modules/canbus_vehicle/lexus/protocol/dash_controls_left_cmd_10c.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Dashcontrolsleftcmd10c::Dashcontrolsleftcmd10c() {}
const int32_t Dashcontrolsleftcmd10c::ID = 0x10C;

void Dashcontrolsleftcmd10c::Parse(const std::uint8_t* bytes, int32_t length,
                                   Lexus* chassis) const {
  chassis->mutable_dash_controls_left_cmd_10c()->set_ignore_overrides(
      ignore_overrides(bytes, length));
  chassis->mutable_dash_controls_left_cmd_10c()->set_enable(
      enable(bytes, length));
  chassis->mutable_dash_controls_left_cmd_10c()->set_clear_override(
      clear_override(bytes, length));
  chassis->mutable_dash_controls_left_cmd_10c()->set_clear_faults(
      clear_faults(bytes, length));
  chassis->mutable_dash_controls_left_cmd_10c()->set_dash_controls_button(
      dash_controls_button(bytes, length));
}

// config detail: {'name': 'ignore_overrides', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Dashcontrolsleftcmd10c::ignore_overrides(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'enable', 'offset': 0.0, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Dashcontrolsleftcmd10c::enable(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'clear_override', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Dashcontrolsleftcmd10c::clear_override(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'clear_faults', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 3,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Dashcontrolsleftcmd10c::clear_faults(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'dash_controls_button', 'enum': {0:
// 'DASH_CONTROLS_BUTTON_DASH_CONTROL_NONE', 1:
// 'DASH_CONTROLS_BUTTON_DASH_CONTROL_OK', 2:
// 'DASH_CONTROLS_BUTTON_DASH_CONTROL_LEFT', 3:
// 'DASH_CONTROLS_BUTTON_DASH_CONTROL_RIGHT', 4:
// 'DASH_CONTROLS_BUTTON_DASH_CONTROL_UP', 5:
// 'DASH_CONTROLS_BUTTON_DASH_CONTROL_DOWN'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit':
// 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Dash_controls_left_cmd_10c::Dash_controls_buttonType
Dashcontrolsleftcmd10c::dash_controls_button(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Dash_controls_left_cmd_10c::Dash_controls_buttonType ret =
      static_cast<Dash_controls_left_cmd_10c::Dash_controls_buttonType>(x);
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
