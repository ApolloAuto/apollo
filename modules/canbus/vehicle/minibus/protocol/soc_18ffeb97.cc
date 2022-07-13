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

#include "modules/canbus/vehicle/minibus/protocol/soc_18ffeb97.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

Soc18ffeb97::Soc18ffeb97() {}
const int32_t Soc18ffeb97::ID = 0x38ffeb97;

void Soc18ffeb97::Parse(const std::uint8_t* bytes, int32_t length,
                        ChassisDetail* chassis) const {
  chassis->mutable_minibus()->mutable_soc_18ffeb97()->set_soc(
      soc(bytes, length));
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name': 'soc',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit':
// '', 'precision': 0.4, 'type': 'double'}
double Soc18ffeb97::soc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.400000;
  return ret;
}
}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
