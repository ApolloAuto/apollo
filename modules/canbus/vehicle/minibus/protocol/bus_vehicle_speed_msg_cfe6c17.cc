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

#include "modules/canbus/vehicle/minibus/protocol/bus_vehicle_speed_msg_cfe6c17.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

Busvehiclespeedmsgcfe6c17::Busvehiclespeedmsgcfe6c17() {}
const int32_t Busvehiclespeedmsgcfe6c17::ID = 0x2cfe6c17;

void Busvehiclespeedmsgcfe6c17::Parse(const std::uint8_t* bytes, int32_t length,
                                      ChassisDetail* chassis) const {
  chassis->mutable_minibus()
      ->mutable_bus_vehicle_speed_msg_cfe6c17()
      ->set_bus_vehicle_speed(bus_vehicle_speed(bytes, length));
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 16, 'name':
// 'bus_vehicle_speed', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': 'Km/h', 'precision': 0.00390625, 'type': 'double'}
double Busvehiclespeedmsgcfe6c17::bus_vehicle_speed(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.003906 / 3.6;  // change km/h to m/s
  return ret;
}
}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
