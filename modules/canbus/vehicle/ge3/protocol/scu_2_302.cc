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

#include "modules/canbus/vehicle/ge3/protocol/scu_2_302.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

Scu2302::Scu2302() {}
const int32_t Scu2302::ID = 0x302;

void Scu2302::Parse(const std::uint8_t* bytes, int32_t length,
                    ChassisDetail* chassis) const {
  chassis->mutable_ge3()->mutable_scu_2_302()->set_vin07(vin07(bytes, length));
  chassis->mutable_ge3()->mutable_scu_2_302()->set_vin06(vin06(bytes, length));
  chassis->mutable_ge3()->mutable_scu_2_302()->set_vin05(vin05(bytes, length));
  chassis->mutable_ge3()->mutable_scu_2_302()->set_vin04(vin04(bytes, length));
  chassis->mutable_ge3()->mutable_scu_2_302()->set_vin03(vin03(bytes, length));
  chassis->mutable_ge3()->mutable_scu_2_302()->set_vin02(vin02(bytes, length));
  chassis->mutable_ge3()->mutable_scu_2_302()->set_vin01(vin01(bytes, length));
  chassis->mutable_ge3()->mutable_scu_2_302()->set_vin00(vin00(bytes, length));
}

// config detail: {'description': 'VIN string character 07', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin07', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu2302::vin07(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'VIN string character 06', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin06', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 55, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu2302::vin06(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'VIN string character 05', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin05', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 47, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu2302::vin05(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'VIN string character 04', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin04', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 39, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu2302::vin04(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'VIN string character 03', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin03', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 31, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu2302::vin03(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'VIN string character 02', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin02', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 23, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu2302::vin02(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'VIN string character 01', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin01', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 15, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu2302::vin01(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'VIN string character 00', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin00', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu2302::vin00(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
