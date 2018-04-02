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

#include "modules/canbus/vehicle/gem/protocol/headlight_cmd_76.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Headlightcmd76::Headlightcmd76() {}
const int32_t Headlightcmd76::ID = 0x76;

void Headlightcmd76::Parse(const std::uint8_t* bytes, int32_t length,
                           ChassisDetail* chassis) const {
  chassis->mutable_gem()->mutable_headlight_cmd_76()->set_headlight_cmd(
      headlight_cmd(bytes, length));
}

// config detail: {'name': 'headlight_cmd', 'enum': {0:
// 'HEADLIGHT_CMD_HEADLIGHTS_OFF', 1: 'HEADLIGHT_CMD_LOW_BEAMS', 2:
// 'HEADLIGHT_CMD_HIGH_BEAMS'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 7, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Headlight_cmd_76::Headlight_cmdType Headlightcmd76::headlight_cmd(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Headlight_cmd_76::Headlight_cmdType ret =
      static_cast<Headlight_cmd_76::Headlight_cmdType>(x);
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
