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

#include "modules/canbus/vehicle/esr/protocol/esr_trackmotionpower_540.h"


#include "glog/logging.h"

#include "modules/canbus/common/byte.h"
#include "modules/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

Esrtrackmotionpower540::Esrtrackmotionpower540() {}
const int32_t Esrtrackmotionpower540::ID = 0x540;

void Esrtrackmotionpower540::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_esr()->mutable_esr_trackmotionpower_540()->set_can_tx_track_rolling_count_2(can_tx_track_rolling_count_2(bytes, length));
}

// config detail: {'name': 'can_tx_track_rolling_count_2', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 4, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Esrtrackmotionpower540::can_tx_track_rolling_count_2(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
