/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/neolix_edu/protocol/aeb_diagnosis1_626.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

Aebdiagnosis1626::Aebdiagnosis1626() {}
const int32_t Aebdiagnosis1626::ID = 0x626;

void Aebdiagnosis1626::Parse(const std::uint8_t* bytes, int32_t length,
                             Neolix_edu* chassis) const {
  chassis->mutable_aeb_diagnosis1_626()->set_aeb_softwareversion(
      aeb_softwareversion(bytes, length));
  chassis->mutable_aeb_diagnosis1_626()->set_aeb_hardwareversion(
      aeb_hardwareversion(bytes, length));
}

// config detail: {'name': 'aeb_softwareversion', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0.0|255.0]', 'bit': 55, 'type': 'double', 'order': 'motorola',
// 'physical_unit': 'bit'}
double Aebdiagnosis1626::aeb_softwareversion(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  double ret = x;
  return ret;
}

// config detail: {'name': 'aeb_hardwareversion', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0.0|255.0]', 'bit': 63, 'type': 'double', 'order': 'motorola',
// 'physical_unit': 'bit'}
double Aebdiagnosis1626::aeb_hardwareversion(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  double ret = x;
  return ret;
}
}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
