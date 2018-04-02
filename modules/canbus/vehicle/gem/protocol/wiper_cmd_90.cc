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

#include "modules/canbus/vehicle/gem/protocol/wiper_cmd_90.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Wipercmd90::Wipercmd90() {}
const int32_t Wipercmd90::ID = 0x90;

void Wipercmd90::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_gem()->mutable_wiper_cmd_90()->set_wiper_cmd(wiper_cmd(bytes, length));
}

// config detail: {'name': 'wiper_cmd', 'enum': {0: 'WIPER_CMD_WIPERS_OFF', 1: 'WIPER_CMD_INTERMITTENT_1', 2: 'WIPER_CMD_INTERMITTENT_2', 3: 'WIPER_CMD_INTERMITTENT_3', 4: 'WIPER_CMD_INTERMITTENT_4', 5: 'WIPER_CMD_INTERMITTENT_5', 6: 'WIPER_CMD_LOW', 7: 'WIPER_CMD_HIGH'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Wiper_cmd_90::Wiper_cmdType Wipercmd90::wiper_cmd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Wiper_cmd_90::Wiper_cmdType ret =  static_cast<Wiper_cmd_90::Wiper_cmdType>(x);
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
