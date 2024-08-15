/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/demo/protocol/vin_resp2_515.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace demo {

using ::apollo::drivers::canbus::Byte;

Vinresp2515::Vinresp2515() {}
const int32_t Vinresp2515::ID = 0x515;

void Vinresp2515::Parse(const std::uint8_t* bytes, int32_t length,
                        Demo* chassis) const {
  chassis->mutable_vin_resp2_515()->set_vin15(vin15(bytes, length));
  chassis->mutable_vin_resp2_515()->set_vin14(vin14(bytes, length));
  chassis->mutable_vin_resp2_515()->set_vin13(vin13(bytes, length));
  chassis->mutable_vin_resp2_515()->set_vin12(vin12(bytes, length));
  chassis->mutable_vin_resp2_515()->set_vin11(vin11(bytes, length));
  chassis->mutable_vin_resp2_515()->set_vin10(vin10(bytes, length));
  chassis->mutable_vin_resp2_515()->set_vin09(vin09(bytes, length));
  chassis->mutable_vin_resp2_515()->set_vin08(vin08(bytes, length));
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'vin15',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vinresp2515::vin15(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 55, 'is_signed_var': False, 'len': 8, 'name': 'vin14',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vinresp2515::vin14(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name': 'vin13',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vinresp2515::vin13(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': False, 'len': 8, 'name': 'vin12',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vinresp2515::vin12(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 31, 'is_signed_var': False, 'len': 8, 'name': 'vin11',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vinresp2515::vin11(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': False, 'len': 8, 'name': 'vin10',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vinresp2515::vin10(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name': 'vin09',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vinresp2515::vin09(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'vin08',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vinresp2515::vin08(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace demo
}  // namespace canbus
}  // namespace apollo
