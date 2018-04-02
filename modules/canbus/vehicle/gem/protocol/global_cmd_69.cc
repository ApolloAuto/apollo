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

#include "modules/canbus/vehicle/gem/protocol/global_cmd_69.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Globalcmd69::Globalcmd69() {}
const int32_t Globalcmd69::ID = 0x69;

void Globalcmd69::Parse(const std::uint8_t* bytes, int32_t length,
                        ChassisDetail* chassis) const {
  chassis->mutable_gem()->mutable_global_cmd_69()->set_pacmod_enable(
      pacmod_enable(bytes, length));
  chassis->mutable_gem()->mutable_global_cmd_69()->set_clear_override(
      clear_override(bytes, length));
  chassis->mutable_gem()->mutable_global_cmd_69()->set_ignore_override(
      ignore_override(bytes, length));
}

// config detail: {'name': 'pacmod_enable', 'enum': {0:
// 'PACMOD_ENABLE_CONTROL_DISABLED', 1: 'PACMOD_ENABLE_CONTROL_ENABLED'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Global_cmd_69::Pacmod_enableType Globalcmd69::pacmod_enable(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Global_cmd_69::Pacmod_enableType ret =
      static_cast<Global_cmd_69::Pacmod_enableType>(x);
  return ret;
}

// config detail: {'name': 'clear_override', 'enum': {0:
// 'CLEAR_OVERRIDE_DON_T_CLEAR_ACTIVE_OVERRIDES', 1:
// 'CLEAR_OVERRIDE_CLEAR_ACTIVE_OVERRIDES'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Global_cmd_69::Clear_overrideType Globalcmd69::clear_override(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  Global_cmd_69::Clear_overrideType ret =
      static_cast<Global_cmd_69::Clear_overrideType>(x);
  return ret;
}

// config detail: {'name': 'ignore_override', 'enum': {0:
// 'IGNORE_OVERRIDE_DON_T_IGNORE_USER_OVERRIDES', 1:
// 'IGNORE_OVERRIDE_IGNORE_USER_OVERRIDES'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 2,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Global_cmd_69::Ignore_overrideType Globalcmd69::ignore_override(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  Global_cmd_69::Ignore_overrideType ret =
      static_cast<Global_cmd_69::Ignore_overrideType>(x);
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
