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

#include "modules/canbus_vehicle/ch/protocol/gear_status_514.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Gearstatus514::Gearstatus514() {}
const int32_t Gearstatus514::ID = 0x514;

void Gearstatus514::Parse(const std::uint8_t* bytes, int32_t length,
                          Ch* chassis) const {
  chassis->mutable_gear_status_514()->set_gear_sts(
      gear_sts(bytes, length));
}

// config detail: {'bit': 0, 'description': 'PRND control(Status)', 'enum': {1:
// 'GEAR_STS_PARK', 2: 'GEAR_STS_REVERSE', 3: 'GEAR_STS_NEUTRAL', 4:
// 'GEAR_STS_DRIVE'}, 'is_signed_var': False, 'len': 8, 'name': 'gear_sts',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[1|4]', 'physical_unit':
// '', 'precision': 1.0, 'type': 'enum'}
Gear_status_514::Gear_stsType Gearstatus514::gear_sts(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Gear_status_514::Gear_stsType ret =
      static_cast<Gear_status_514::Gear_stsType>(x);
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
