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

#include "modules/canbus/vehicle/lincoln/protocol/gear_66.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

// public
const int32_t Gear66::ID = 0x66;

uint32_t Gear66::GetPeriod() const {
  // on event, so value nonsense
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Gear66::UpdateData(uint8_t *data) {
  set_gear_p(data, gear_);
  set_clear_driver_override_flag_p(data);
}

void Gear66::Reset() { gear_ = 0; }

Gear66 *Gear66::set_gear_none() {
  gear_ = 0x00;
  return this;
}

Gear66 *Gear66::set_gear_park() {
  gear_ = 0x01;
  return this;
}

Gear66 *Gear66::set_gear_reverse() {
  gear_ = 0x02;
  return this;
}

Gear66 *Gear66::set_gear_neutral() {
  gear_ = 0x03;
  return this;
}

Gear66 *Gear66::set_gear_drive() {
  gear_ = 0x04;
  return this;
}

Gear66 *Gear66::set_gear_low() {
  gear_ = 0x05;
  return this;
}

// private
void Gear66::set_gear_p(uint8_t *data, int32_t gear) {
  gear = ProtocolData::BoundedValue(0, 5, gear);
  Byte frame(data);
  frame.set_value(static_cast<uint8_t>(gear), 0, 3);
}

void Gear66::set_clear_driver_override_flag_p(uint8_t *bytes) {
  Byte frame(bytes);
  frame.set_bit_0(7);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
