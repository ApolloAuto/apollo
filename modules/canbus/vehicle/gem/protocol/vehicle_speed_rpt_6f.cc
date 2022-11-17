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

#include "modules/canbus/vehicle/gem/protocol/vehicle_speed_rpt_6f.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Vehiclespeedrpt6f::Vehiclespeedrpt6f() {}
const int32_t Vehiclespeedrpt6f::ID = 0x6F;

void Vehiclespeedrpt6f::Parse(const std::uint8_t* bytes, int32_t length,
                              ChassisDetail* chassis) const {
  chassis->mutable_gem()->mutable_vehicle_speed_rpt_6f()->set_vehicle_speed(
      vehicle_speed(bytes, length));
  chassis->mutable_gem()
      ->mutable_vehicle_speed_rpt_6f()
      ->set_vehicle_speed_valid(vehicle_speed_valid(bytes, length));
}

// config detail: {'name': 'vehicle_speed', 'offset': 0.0, 'precision': 0.01,
// 'len': 16, 'is_signed_var': True, 'physical_range': '[-327.68|327.67]',
// 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
double Vehiclespeedrpt6f::vehicle_speed(const std::uint8_t* bytes,
                                        int32_t length) const {
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

// config detail: {'name': 'vehicle_speed_valid', 'enum': {0:
// 'VEHICLE_SPEED_VALID_INVALID', 1: 'VEHICLE_SPEED_VALID_VALID'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Vehicle_speed_rpt_6f::Vehicle_speed_validType
Vehiclespeedrpt6f::vehicle_speed_valid(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  Vehicle_speed_rpt_6f::Vehicle_speed_validType ret =
      static_cast<Vehicle_speed_rpt_6f::Vehicle_speed_validType>(x);
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
