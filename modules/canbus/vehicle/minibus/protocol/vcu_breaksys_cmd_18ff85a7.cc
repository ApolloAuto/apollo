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

#include "modules/canbus/vehicle/minibus/protocol/vcu_breaksys_cmd_18ff85a7.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

Vcubreaksyscmd18ff85a7::Vcubreaksyscmd18ff85a7() {}
const int32_t Vcubreaksyscmd18ff85a7::ID = 0x38ff85a7;

void Vcubreaksyscmd18ff85a7::Parse(const std::uint8_t* bytes, int32_t length,
                                   ChassisDetail* chassis) const {
  chassis->mutable_minibus()
      ->mutable_vcu_breaksys_cmd_18ff85a7()
      ->set_vcu_brk_autoparking_request(
          vcu_brk_autoparking_request(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_breaksys_cmd_18ff85a7()
      ->set_vcu_brk_initivate_enable(vcu_brk_initivate_enable(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_breaksys_cmd_18ff85a7()
      ->set_vcu_brk_right_pressure(vcu_brk_right_pressure(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_breaksys_cmd_18ff85a7()
      ->set_vcu_brk_left_pressure(vcu_brk_left_pressure(bytes, length));
}

// config detail: {'bit': 23, 'is_signed_var': False, 'len': 1, 'name':
// 'vcu_brk_autoparking_request', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'bool'}
bool Vcubreaksyscmd18ff85a7::vcu_brk_autoparking_request(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 22, 'is_signed_var': False, 'len': 1, 'name':
// 'vcu_brk_initivate_enable', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'bool'}
bool Vcubreaksyscmd18ff85a7::vcu_brk_initivate_enable(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
// 'vcu_brk_right_pressure', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|25.5]', 'physical_unit': 'MPa', 'precision': 0.1, 'type': 'double'}
double Vcubreaksyscmd18ff85a7::vcu_brk_right_pressure(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name':
// 'vcu_brk_left_pressure', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|25.5]', 'physical_unit': 'MPa', 'precision': 0.1, 'type': 'double'}
double Vcubreaksyscmd18ff85a7::vcu_brk_left_pressure(const std::uint8_t* bytes,
                                                     int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.100000;
  return ret;
}
}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
