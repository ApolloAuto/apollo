/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/lexus/protocol/wiper_cmd_134.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

const int32_t Wipercmd134::ID = 0x134;

// public
Wipercmd134::Wipercmd134() { Reset(); }

uint32_t Wipercmd134::GetPeriod() const {
  // TODO(QiL) modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Wipercmd134::UpdateData(uint8_t* data) {
  set_p_ignore_overrides(data, ignore_overrides_);
  set_p_enable(data, enable_);
  set_p_clear_override(data, clear_override_);
  set_p_wiper_cmd(data, wiper_cmd_);
  set_p_clear_faults(data, clear_faults_);
}

void Wipercmd134::Reset() {
  // TODO(QiL) you should check this manually
  ignore_overrides_ = false;
  enable_ = false;
  clear_override_ = false;
  wiper_cmd_ = Wiper_cmd_134::WIPER_CMD_WIPERS_OFF;
  clear_faults_ = false;
}

Wipercmd134* Wipercmd134::set_ignore_overrides(bool ignore_overrides) {
  ignore_overrides_ = ignore_overrides;
  return this;
}

// config detail: {'name': 'IGNORE_OVERRIDES', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
void Wipercmd134::set_p_ignore_overrides(uint8_t* data, bool ignore_overrides) {
  uint8_t x = ignore_overrides;

  Byte to_set(data + 0);
  to_set.set_value(x, 1, 1);
}

Wipercmd134* Wipercmd134::set_enable(bool enable) {
  enable_ = enable;
  return this;
}

// config detail: {'name': 'ENABLE', 'offset': 0.0, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
void Wipercmd134::set_p_enable(uint8_t* data, bool enable) {
  uint8_t x = enable;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

Wipercmd134* Wipercmd134::set_clear_override(bool clear_override) {
  clear_override_ = clear_override;
  return this;
}

// config detail: {'name': 'CLEAR_OVERRIDE', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
void Wipercmd134::set_p_clear_override(uint8_t* data, bool clear_override) {
  uint8_t x = clear_override;

  Byte to_set(data + 0);
  to_set.set_value(x, 2, 1);
}

Wipercmd134* Wipercmd134::set_wiper_cmd(
    Wiper_cmd_134::Wiper_cmdType wiper_cmd) {
  wiper_cmd_ = wiper_cmd;
  return this;
}

// config detail: {'name': 'WIPER_CMD', 'enum': {0: 'WIPER_CMD_WIPERS_OFF', 1:
// 'WIPER_CMD_INTERMITTENT_1', 2: 'WIPER_CMD_INTERMITTENT_2', 3:
// 'WIPER_CMD_INTERMITTENT_3', 4: 'WIPER_CMD_INTERMITTENT_4', 5:
// 'WIPER_CMD_INTERMITTENT_5', 6: 'WIPER_CMD_LOW', 7: 'WIPER_CMD_HIGH'},
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|7]', 'bit': 15, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Wipercmd134::set_p_wiper_cmd(uint8_t* data,
                                  Wiper_cmd_134::Wiper_cmdType wiper_cmd) {
  uint8_t x = wiper_cmd;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}

Wipercmd134* Wipercmd134::set_clear_faults(bool clear_faults) {
  clear_faults_ = clear_faults;
  return this;
}

// config detail: {'name': 'CLEAR_FAULTS', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 3,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
void Wipercmd134::set_p_clear_faults(uint8_t* data, bool clear_faults) {
  uint8_t x = clear_faults;

  Byte to_set(data + 0);
  to_set.set_value(x, 3, 1);
}

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
