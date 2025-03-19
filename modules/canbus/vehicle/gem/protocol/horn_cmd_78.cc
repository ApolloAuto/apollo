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

#include "modules/canbus/vehicle/gem/protocol/horn_cmd_78.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

const int32_t Horncmd78::ID = 0x78;

// public
Horncmd78::Horncmd78() { Reset(); }

uint32_t Horncmd78::GetPeriod() const {
  // TODO(QiL) :modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Horncmd78::UpdateData(uint8_t* data) { set_p_horn_cmd(data, horn_cmd_); }

void Horncmd78::Reset() {
  // TODO(QiL) :you should check this manually
  horn_cmd_ = Horn_cmd_78::HORN_CMD_OFF;
}

Horncmd78* Horncmd78::set_horn_cmd(Horn_cmd_78::Horn_cmdType horn_cmd) {
  horn_cmd_ = horn_cmd;
  return this;
}

// config detail: {'name': 'HORN_CMD', 'enum': {0: 'HORN_CMD_OFF', 1:
// 'HORN_CMD_ON'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset':
// 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
void Horncmd78::set_p_horn_cmd(uint8_t* data,
                               Horn_cmd_78::Horn_cmdType horn_cmd) {
  uint8_t x = horn_cmd;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
