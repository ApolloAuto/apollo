/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/ch/protocol/turnsignal_status__513.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Turnsignalstatus513::Turnsignalstatus513() {}
const int32_t Turnsignalstatus513::ID = 0x513;

void Turnsignalstatus513::Parse(const std::uint8_t* bytes, int32_t length,
                                ChassisDetail* chassis) const {
  chassis->mutable_ch()->mutable_turnsignal_status__513()->set_turn_signal_sts(
      turn_signal_sts(bytes, length));
}

// config detail: {'description': 'Lighting control(Status)', 'enum': {0:
// 'TURN_SIGNAL_STS_NONE', 1: 'TURN_SIGNAL_STS_LEFT', 2:
// 'TURN_SIGNAL_STS_RIGHT'}, 'precision': 1.0, 'len': 8, 'name':
// 'turn_signal_sts', 'is_signed_var': False, 'offset': 0.0, 'physical_range':
// '[0|2]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Turnsignal_status__513::Turn_signal_stsType
Turnsignalstatus513::turn_signal_sts(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  return static_cast<Turnsignal_status__513::Turn_signal_stsType>(x);
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
