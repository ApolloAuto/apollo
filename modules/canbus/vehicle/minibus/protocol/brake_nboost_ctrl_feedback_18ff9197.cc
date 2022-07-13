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

#include "modules/canbus/vehicle/minibus/protocol/brake_nboost_ctrl_feedback_18ff9197.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

Brakenboostctrlfeedback18ff9197::Brakenboostctrlfeedback18ff9197() {}
const int32_t Brakenboostctrlfeedback18ff9197::ID = 0x38ff9197;

void Brakenboostctrlfeedback18ff9197::Parse(const std::uint8_t* bytes,
                                            int32_t length,
                                            ChassisDetail* chassis) const {
  chassis->mutable_minibus()
      ->mutable_brake_nboost_ctrl_feedback_18ff9197()
      ->set_aeb_brkpelposdes(aeb_brkpelposdes(bytes, length));
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name':
// 'aeb_brkpelposdes', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|100]', 'physical_unit': '', 'precision': 0.01, 'type': 'double'}
double Brakenboostctrlfeedback18ff9197::aeb_brkpelposdes(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}
}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
