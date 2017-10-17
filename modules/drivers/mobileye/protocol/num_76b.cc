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

#include "modules/drivers/mobileye/protocol/num_76b.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace mobileye {

using ::apollo::drivers::canbus::Byte;

const int Num76b::ID = 0x76b;

void Num76b::Parse(const uint8_t* bytes, int32_t length,
                   Mobileye* mobileye) const {
  mobileye->mutable_num_76b()->set_num_of_next_lane_mark_reported(
      num_of_next_lane_mark_reported(bytes, length));
  mobileye->mutable_next_76c()->Clear();
  mobileye->mutable_next_76d()->Clear();
}

// config detail: {'name': 'num_of_next_lane_mark_reported', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'f_type': 'value', 'is_signed_var': False,
// 'physical_range': '[0|0]', 'bit': 7, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '"Number  apollo  of  apollo  next  apollo  lane  apollo
// markers"'}
int Num76b::num_of_next_lane_mark_reported(const uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  return x;
}

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo
