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

#include "modules/canbus/vehicle/hunter2/protocol/mileage_feedback_311.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

using ::apollo::drivers::canbus::Byte;

Mileagefeedback311::Mileagefeedback311() {}
const int32_t Mileagefeedback311::ID = 0x311;

void Mileagefeedback311::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_hunter2()->mutable_mileage_feedback_311()->set_rightback_milemeter(rightback_milemeter(bytes, length));
  chassis->mutable_hunter2()->mutable_mileage_feedback_311()->set_leftback_milemeter(leftback_milemeter(bytes, length));
}

// config detail: {'bit': 39, 'is_signed_var': True, 'len': 32, 'name': 'rightback_milemeter', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-2147483648|2147483647]', 'physical_unit': 'mm', 'precision': 1.0, 'type': 'int'}
int Mileagefeedback311::rightback_milemeter(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 6);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 7);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 0;
  x >>= 0;

  int ret = x;
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': True, 'len': 32, 'name': 'leftback_milemeter', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-2147483648|2147483647]', 'physical_unit': 'mm', 'precision': 1.0, 'type': 'int'}
int Mileagefeedback311::leftback_milemeter(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 2);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 3);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 0;
  x >>= 0;

  int ret = x;
  return ret;
}
}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo
