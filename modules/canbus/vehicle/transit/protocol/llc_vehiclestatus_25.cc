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

#include "modules/canbus/vehicle/transit/protocol/llc_vehiclestatus_25.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

Llcvehiclestatus25::Llcvehiclestatus25() {}
const int32_t Llcvehiclestatus25::ID = 0x25;

void Llcvehiclestatus25::Parse(const std::uint8_t* bytes, int32_t length,
                               ChassisDetail* chassis) const {
  chassis->mutable_transit()
      ->mutable_llc_vehiclestatus_25()
      ->set_llc_fbk_12voltage(llc_fbk_12voltage(bytes, length));
}

// config detail: {'description': 'Vehicle 12V voltage feedback', 'offset': 0.0,
// 'precision': 0.1, 'len': 8, 'name': 'llc_fbk_12voltage', 'is_signed_var':
// False, 'physical_range': '[0|25.5]', 'bit': 0, 'type': 'double', 'order':
// 'intel', 'physical_unit': 'Volt'}
double Llcvehiclestatus25::llc_fbk_12voltage(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.100000;
  return ret;
}
}  // namespace transit
}  // namespace canbus
}  // namespace apollo
