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

#include "modules/canbus/vehicle/lexus/protocol/horn_cmd_11c.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Horncmd11c::Horncmd11c() {}
const int32_t Horncmd11c::ID = 0x11C;

void Horncmd11c::Parse(const std::uint8_t* bytes, int32_t length,
                       ChassisDetail* chassis) const {
  chassis->mutable_lexus()->mutable_horn_cmd_11c()->set_ignore_overrides(
      ignore_overrides(bytes, length));
  chassis->mutable_lexus()->mutable_horn_cmd_11c()->set_enable(
      enable(bytes, length));
  chassis->mutable_lexus()->mutable_horn_cmd_11c()->set_clear_override(
      clear_override(bytes, length));
  chassis->mutable_lexus()->mutable_horn_cmd_11c()->set_clear_faults(
      clear_faults(bytes, length));
  chassis->mutable_lexus()->mutable_horn_cmd_11c()->set_horn_cmd(
      horn_cmd(bytes, length));
}

// config detail: {'name': 'ignore_overrides', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Horncmd11c::ignore_overrides(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'enable', 'offset': 0.0, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Horncmd11c::enable(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'clear_override', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Horncmd11c::clear_override(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'clear_faults', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 3,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Horncmd11c::clear_faults(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'horn_cmd', 'enum': {0: 'HORN_CMD_OFF', 1:
// 'HORN_CMD_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset':
// 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Horn_cmd_11c::Horn_cmdType Horncmd11c::horn_cmd(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  Horn_cmd_11c::Horn_cmdType ret = static_cast<Horn_cmd_11c::Horn_cmdType>(x);
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
