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

#include "modules/canbus_vehicle/devkit/protocol/vin_resp3_516.h"

#include <string>

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::drivers::canbus::Byte;

Vinresp3516::Vinresp3516() {}
const int32_t Vinresp3516::ID = 0x516;

void Vinresp3516::Parse(const std::uint8_t* bytes, int32_t length,
                        Devkit* chassis) const {
  chassis->mutable_vin_resp3_516()->set_vin16(
      vin16(bytes, length));
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'vin16',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
std::string Vinresp3516::vin16(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  std::string ret = "";
  ret += x;
  return ret;
}
}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
