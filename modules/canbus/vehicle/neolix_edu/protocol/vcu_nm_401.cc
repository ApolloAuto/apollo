/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/neolix_edu/protocol/vcu_nm_401.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

Vcunm401::Vcunm401() {}
const int32_t Vcunm401::ID = 0x401;

void Vcunm401::Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const {
  chassis->mutable_neolix_edu()->mutable_vcu_nm_401()->set_vcu_sleepcommand(
      vcu_sleepcommand(bytes, length));
}

// config detail: {'description': '0x0:Inactive;0x1:Active', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'vcu_sleepcommand', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 0, 'type': 'bool', 'order':
// 'motorola', 'physical_unit': 'bit'}
bool Vcunm401::vcu_sleepcommand(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
