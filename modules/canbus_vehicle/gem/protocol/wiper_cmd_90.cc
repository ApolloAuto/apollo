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

#include "modules/canbus/vehicle/gem/protocol/wiper_cmd_90.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

const int32_t Wipercmd90::ID = 0x90;

// public
Wipercmd90::Wipercmd90() { Reset(); }

uint32_t Wipercmd90::GetPeriod() const {
  // TODO(QiL) :modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Wipercmd90::UpdateData(uint8_t* data) {
  set_p_wiper_cmd(data, wiper_cmd_);
}

void Wipercmd90::Reset() {
  // TODO(QiL) :you should check this manually
  wiper_cmd_ = Wiper_cmd_90::WIPER_CMD_WIPERS_OFF;
}

Wipercmd90* Wipercmd90::set_wiper_cmd(Wiper_cmd_90::Wiper_cmdType wiper_cmd) {
  wiper_cmd_ = wiper_cmd;
  return this;
}

// config detail: {'name': 'WIPER_CMD', 'enum': {0: 'WIPER_CMD_WIPERS_OFF', 1:
// 'WIPER_CMD_INTERMITTENT_1', 2: 'WIPER_CMD_INTERMITTENT_2', 3:
// 'WIPER_CMD_INTERMITTENT_3', 4: 'WIPER_CMD_INTERMITTENT_4', 5:
// 'WIPER_CMD_INTERMITTENT_5', 6: 'WIPER_CMD_LOW', 7: 'WIPER_CMD_HIGH'},
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|7]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Wipercmd90::set_p_wiper_cmd(uint8_t* data,
                                 Wiper_cmd_90::Wiper_cmdType wiper_cmd) {
  uint8_t x = wiper_cmd;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
