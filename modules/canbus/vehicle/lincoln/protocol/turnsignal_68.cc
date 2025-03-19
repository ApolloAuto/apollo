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

#include "modules/canbus/vehicle/lincoln/protocol/turnsignal_68.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Turnsignal68::ID = 0x68;

uint32_t Turnsignal68::GetPeriod() const {
  static const uint32_t PERIOD = 50 * 1000;
  return PERIOD;
}

int32_t Turnsignal68::turn_cmd() const { return turn_cmd_; }

void Turnsignal68::UpdateData(uint8_t *data) {
  set_turn_cmd_p(data, turn_cmd_);
}

void Turnsignal68::Reset() { turn_cmd_ = 0; }

Turnsignal68 *Turnsignal68::set_turn_none() {
  turn_cmd_ = 0x00;
  return this;
}

Turnsignal68 *Turnsignal68::set_turn_left() {
  turn_cmd_ = 0x01;
  return this;
}

Turnsignal68 *Turnsignal68::set_turn_right() {
  turn_cmd_ = 0x02;
  return this;
}
// 0x03 not used

// private
void Turnsignal68::set_turn_cmd_p(uint8_t *data, int32_t turn_cmd) {
  turn_cmd = ProtocolData::BoundedValue(0, 3, turn_cmd);
  Byte frame(data + 0);
  frame.set_value(static_cast<uint8_t>(turn_cmd), 0, 2);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
