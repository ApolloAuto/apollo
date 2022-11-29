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

#include "modules/canbus_vehicle/ch/protocol/turnsignal_status__513.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Turnsignalstatus513::Turnsignalstatus513() {}
const int32_t Turnsignalstatus513::ID = 0x513;

void Turnsignalstatus513::Parse(const std::uint8_t* bytes, int32_t length,
                                Ch* chassis) const {
  chassis->mutable_turnsignal_status__513()->set_turn_signal_sts(
      turn_signal_sts(bytes, length));
  chassis->mutable_turnsignal_status__513()->set_low_beam_sts(
      low_beam_sts(bytes, length));
}

// config detail: {'bit': 0, 'description': 'Lighting control(Status)', 'enum':
// {0: 'TURN_SIGNAL_STS_NONE', 1: 'TURN_SIGNAL_STS_LEFT', 2:
// 'TURN_SIGNAL_STS_RIGHT', 3: 'TURN_SIGNAL_STS_HAZARD_WARNING_LAMPSTS_ON'},
// 'is_signed_var': False, 'len': 8, 'name': 'turn_signal_sts', 'offset': 0.0,
// 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'enum'}
Turnsignal_status__513::Turn_signal_stsType
Turnsignalstatus513::turn_signal_sts(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Turnsignal_status__513::Turn_signal_stsType ret =
      static_cast<Turnsignal_status__513::Turn_signal_stsType>(x);
  return ret;
}

// config detail: {'bit': 8, 'description': 'Lighting control(Status)', 'enum':
// {0: 'LOW_BEAM_STS_ON', 1: 'LOW_BEAM_STS_OFF'}, 'is_signed_var': False, 'len':
// 2, 'name': 'low_beam_sts', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Turnsignal_status__513::Low_beam_stsType Turnsignalstatus513::low_beam_sts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 2);

  Turnsignal_status__513::Low_beam_stsType ret =
      static_cast<Turnsignal_status__513::Low_beam_stsType>(x);
  return ret;
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
