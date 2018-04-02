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

#include "modules/canbus/vehicle/gem/protocol/shift_cmd_65.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Shiftcmd65::Shiftcmd65() {}
const int32_t Shiftcmd65::ID = 0x65;

void Shiftcmd65::Parse(const std::uint8_t* bytes, int32_t length,
                       ChassisDetail* chassis) const {
  chassis->mutable_gem()->mutable_shift_cmd_65()->set_shift_cmd(
      shift_cmd(bytes, length));
}

// config detail: {'description':
// 'FORWARD_is_also_LOW_on_vehicles_with_LOW/HIGH,_PARK_and_HIGH_only_available_on_certain_Vehicles',
// 'enum': {0: 'SHIFT_CMD_PARK', 1: 'SHIFT_CMD_REVERSE', 2: 'SHIFT_CMD_NEUTRAL',
// 3: 'SHIFT_CMD_FORWARD', 4: 'SHIFT_CMD_LOW'}, 'precision': 1.0, 'len': 8,
// 'name': 'shift_cmd', 'is_signed_var': False, 'offset': 0.0, 'physical_range':
// '[0|4]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Shift_cmd_65::Shift_cmdType Shiftcmd65::shift_cmd(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Shift_cmd_65::Shift_cmdType ret = static_cast<Shift_cmd_65::Shift_cmdType>(x);
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
