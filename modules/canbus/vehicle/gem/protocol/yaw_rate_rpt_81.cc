/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/gem/protocol/yaw_rate_rpt_81.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Yawraterpt81::Yawraterpt81() {}
const int32_t Yawraterpt81::ID = 0x81;

void Yawraterpt81::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_gem()->mutable_yaw_rate_rpt_81()->set_yaw_rate(
      yaw_rate(bytes, length));
}

// config detail: {'name': 'yaw_rate', 'offset': 0.0, 'precision': 0.01, 'len':
// 16, 'is_signed_var': True, 'physical_range': '[-327.68|327.67]', 'bit': 7,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'rad/s'}
double Yawraterpt81::yaw_rate(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
