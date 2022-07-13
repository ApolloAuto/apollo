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

#include "modules/canbus/vehicle/minibus/protocol/bus_battery_meg_18fa1017.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

Busbatterymeg18fa1017::Busbatterymeg18fa1017() {}
const int32_t Busbatterymeg18fa1017::ID = 0x38fa1017;

void Busbatterymeg18fa1017::Parse(const std::uint8_t* bytes, int32_t length,
                                  ChassisDetail* chassis) const {
  chassis->mutable_minibus()
      ->mutable_bus_battery_meg_18fa1017()
      ->set_bus_battery_voltage(bus_battery_voltage(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name':
// 'bus_battery_voltage', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 0.05, 'type': 'double'}
double Busbatterymeg18fa1017::bus_battery_voltage(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.050000;
  return ret;
}
}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
