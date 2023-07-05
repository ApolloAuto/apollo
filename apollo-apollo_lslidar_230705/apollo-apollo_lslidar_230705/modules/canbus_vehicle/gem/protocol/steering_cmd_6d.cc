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

#include "modules/canbus_vehicle/gem/protocol/steering_cmd_6d.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

const int32_t Steeringcmd6d::ID = 0x6D;

// public
Steeringcmd6d::Steeringcmd6d() { Reset(); }

uint32_t Steeringcmd6d::GetPeriod() const {
  // TODO(QiL) :modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Steeringcmd6d::UpdateData(uint8_t* data) {
  set_p_position_value(data, position_value_);
  set_p_speed_limit(data, speed_limit_);
}

void Steeringcmd6d::Reset() {
  // TODO(QiL) :you should check this manually
  position_value_ = 0.0;
  speed_limit_ = 0.0;
}

Steeringcmd6d* Steeringcmd6d::set_position_value(double position_value) {
  position_value_ = position_value;
  return this;
}

// config detail: {'name': 'POSITION_VALUE', 'offset': 0.0, 'precision': 0.001,
// 'len': 32, 'is_signed_var': True, 'physical_range':
// '[-2147483.648|2147483.647]', 'bit': 7, 'type': 'double', 'order':
// 'motorola', 'physical_unit': 'radians'}
void Steeringcmd6d::set_p_position_value(uint8_t* data, double position_value) {
  position_value =
      ProtocolData::BoundedValue(-2147483.648, 2147483.647, position_value);
  int x = static_cast<int>(position_value / 0.001000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 3);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 2);
  to_set1.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set2(data + 1);
  to_set2.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set3(data + 0);
  to_set3.set_value(t, 0, 8);
}

Steeringcmd6d* Steeringcmd6d::set_speed_limit(double speed_limit) {
  speed_limit_ = speed_limit;
  return this;
}

// config detail: {'name': 'SPEED_LIMIT', 'offset': 0.0, 'precision': 0.001,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit': 39,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'rad/s'}
void Steeringcmd6d::set_p_speed_limit(uint8_t* data, double speed_limit) {
  speed_limit = ProtocolData::BoundedValue(0.0, 65.535, speed_limit);
  int x = static_cast<int>(speed_limit / 0.001000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 7);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 6);
  to_set1.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set2(data + 5);
  to_set2.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set3(data + 4);
  to_set3.set_value(t, 0, 8);
}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
