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

#include "modules/canbus/vehicle/gem/protocol/accel_cmd_67.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

const int32_t Accelcmd67::ID = 0x67;

// public
Accelcmd67::Accelcmd67() { Reset(); }

uint32_t Accelcmd67::GetPeriod() const {
  // TODO(QiL) :modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Accelcmd67::UpdateData(uint8_t* data) {
  set_p_accel_cmd(data, accel_cmd_);
}

void Accelcmd67::Reset() {
  // TODO(QiL) :you should check this manually
  accel_cmd_ = 0.0;
}

Accelcmd67* Accelcmd67::set_accel_cmd(double accel_cmd) {
  accel_cmd_ = accel_cmd;
  return this;
}

// config detail: {'name': 'ACCEL_CMD', 'offset': 0.0, 'precision': 0.001,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 7,
// 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
void Accelcmd67::set_p_accel_cmd(uint8_t* data, double accel_cmd) {
  accel_cmd = ProtocolData::BoundedValue(0.0, 1.0, accel_cmd);
  int x = accel_cmd / 0.001000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 1);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 0);
  to_set1.set_value(t, 0, 8);
}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
