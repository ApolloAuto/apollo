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

#include "modules/canbus/vehicle/minibus/protocol/bus_mileage_18fee017.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

Busmileage18fee017::Busmileage18fee017() {}
const int32_t Busmileage18fee017::ID = 0x38fee017;

void Busmileage18fee017::Parse(const std::uint8_t* bytes, int32_t length,
                               ChassisDetail* chassis) const {
  chassis->mutable_minibus()
      ->mutable_bus_mileage_18fee017()
      ->set_bus_mileage_long(bus_mileage_long(bytes, length));
  chassis->mutable_minibus()
      ->mutable_bus_mileage_18fee017()
      ->set_bus_mileage_short(bus_mileage_short(bytes, length));
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 32, 'name':
// 'bus_mileage_long', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': 'Km', 'precision': 0.125, 'type': 'double'}
double Busmileage18fee017::bus_mileage_long(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 5);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 4);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.125000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 32, 'name':
// 'bus_mileage_short', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': 'Km', 'precision': 0.125, 'type': 'double'}
double Busmileage18fee017::bus_mileage_short(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 1);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 0);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.125000;
  return ret;
}
}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
