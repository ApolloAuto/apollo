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

#include "modules/canbus/vehicle/ge3/protocol/scu_3_303.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

Scu3303::Scu3303() {}
const int32_t Scu3303::ID = 0x303;

void Scu3303::Parse(const std::uint8_t* bytes, int32_t length,
                    ChassisDetail* chassis) const {
  chassis->mutable_ge3()->mutable_scu_3_303()->set_vin15(vin15(bytes, length));
  chassis->mutable_ge3()->mutable_scu_3_303()->set_vin14(vin14(bytes, length));
  chassis->mutable_ge3()->mutable_scu_3_303()->set_vin13(vin13(bytes, length));
  chassis->mutable_ge3()->mutable_scu_3_303()->set_vin12(vin12(bytes, length));
  chassis->mutable_ge3()->mutable_scu_3_303()->set_vin11(vin11(bytes, length));
  chassis->mutable_ge3()->mutable_scu_3_303()->set_vin10(vin10(bytes, length));
  chassis->mutable_ge3()->mutable_scu_3_303()->set_vin09(vin09(bytes, length));
  chassis->mutable_ge3()->mutable_scu_3_303()->set_vin08(vin08(bytes, length));
}

// config detail: {'description': 'VIN string character 15', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin15', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu3303::vin15(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'VIN string character 14', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin14', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 55, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu3303::vin14(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'VIN string character 13', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin13', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 47, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu3303::vin13(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'VIN string character 12', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin12', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 39, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu3303::vin12(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'VIN string character 11', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin11', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 31, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu3303::vin11(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'VIN string character 10', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin10', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 23, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu3303::vin10(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'VIN string character 09', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin09', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 15, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu3303::vin09(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'VIN string character 08', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'vin08', 'is_signed_var': False,
// 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola',
// 'physical_unit': '-'}
int Scu3303::vin08(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
