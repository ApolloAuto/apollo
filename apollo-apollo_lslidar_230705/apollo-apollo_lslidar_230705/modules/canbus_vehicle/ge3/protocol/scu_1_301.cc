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

#include "modules/canbus_vehicle/ge3/protocol/scu_1_301.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

Scu1301::Scu1301() {}
const int32_t Scu1301::ID = 0x301;

void Scu1301::Parse(const std::uint8_t* bytes, int32_t length,
                    Ge3* chassis) const {
  chassis->mutable_scu_1_301()->set_vin16(vin16(bytes, length));
  chassis->mutable_scu_1_301()->set_scu_stopbutst(
      scu_stopbutst(bytes, length));
  chassis->mutable_scu_1_301()->set_scu_drvmode(
      scu_drvmode(bytes, length));
  chassis->mutable_scu_1_301()->set_scu_faultst(
      scu_faultst(bytes, length));
}

// config detail: {'description': 'VIN string character 16', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin16', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 15, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu1301::vin16(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Brake pedal position', 'enum': {0:
// 'SCU_STOPBUTST_UNPRESSED', 1: 'SCU_STOPBUTST_PRESSED'}, 'precision': 1.0,
// 'len': 1, 'name': 'scu_stopbutst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_1_301::Scu_stopbutstType Scu1301::scu_stopbutst(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Scu_1_301::Scu_stopbutstType ret =
      static_cast<Scu_1_301::Scu_stopbutstType>(x);
  return ret;
}

// config detail: {'description': 'SCU drive mode', 'enum': {0:
// 'SCU_DRVMODE_INVALID', 1: 'SCU_DRVMODE_MANUAL', 2: 'SCU_DRVMODE_INTERRUPT',
// 3: 'SCU_DRVMODE_AUTO'}, 'precision': 1.0, 'len': 2, 'name': 'scu_drvmode',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 3,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Scu_1_301::Scu_drvmodeType Scu1301::scu_drvmode(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 2);

  Scu_1_301::Scu_drvmodeType ret = static_cast<Scu_1_301::Scu_drvmodeType>(x);
  return ret;
}

// config detail: {'description': 'SCU fault status', 'enum': {0:
// 'SCU_FAULTST_NORMAL', 1: 'SCU_FAULTST_FAULT'}, 'precision': 1.0, 'len': 4,
// 'name': 'scu_faultst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|15]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_1_301::Scu_faultstType Scu1301::scu_faultst(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 4);

  Scu_1_301::Scu_faultstType ret = static_cast<Scu_1_301::Scu_faultstType>(x);
  return ret;
}
}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
