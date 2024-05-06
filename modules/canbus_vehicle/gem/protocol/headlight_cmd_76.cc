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

#include "modules/canbus_vehicle/gem/protocol/headlight_cmd_76.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

const int32_t Headlightcmd76::ID = 0x76;

// public
Headlightcmd76::Headlightcmd76() { Reset(); }

uint32_t Headlightcmd76::GetPeriod() const {
  // TODO(QiL) :modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Headlightcmd76::UpdateData(uint8_t* data) {
  set_p_headlight_cmd(data, headlight_cmd_);
}

void Headlightcmd76::Reset() {
  // TODO(QiL) :you should check this manually
  headlight_cmd_ = Headlight_cmd_76::HEADLIGHT_CMD_HEADLIGHTS_OFF;
}

Headlightcmd76* Headlightcmd76::set_headlight_cmd(
    Headlight_cmd_76::Headlight_cmdType headlight_cmd) {
  headlight_cmd_ = headlight_cmd;
  return this;
}

// config detail: {'name': 'HEADLIGHT_CMD', 'enum': {0:
// 'HEADLIGHT_CMD_HEADLIGHTS_OFF', 1: 'HEADLIGHT_CMD_LOW_BEAMS', 2:
// 'HEADLIGHT_CMD_HIGH_BEAMS'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 7, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
void Headlightcmd76::set_p_headlight_cmd(
    uint8_t* data, Headlight_cmd_76::Headlight_cmdType headlight_cmd) {
  uint8_t x = headlight_cmd;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
