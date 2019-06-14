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

#include "modules/canbus/vehicle/ch/protocol/ecu_status_3_517.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Ecustatus3517::Ecustatus3517() {}
const int32_t Ecustatus3517::ID = 0x517;

void Ecustatus3517::Parse(const std::uint8_t* bytes, int32_t length,
                          ChassisDetail* chassis) const {
  chassis->mutable_ch()->mutable_ecu_status_3_517()->set_ultrasound_dist_1(
      ultrasound_dist_1(bytes, length));
  chassis->mutable_ch()->mutable_ecu_status_3_517()->set_ultrasound_dist_2(
      ultrasound_dist_2(bytes, length));
  chassis->mutable_ch()->mutable_ecu_status_3_517()->set_ultrasound_dist_3(
      ultrasound_dist_3(bytes, length));
  chassis->mutable_ch()->mutable_ecu_status_3_517()->set_ultrasound_dist_4(
      ultrasound_dist_4(bytes, length));
  chassis->mutable_ch()->mutable_ecu_status_3_517()->set_ultrasound_dist_5(
      ultrasound_dist_5(bytes, length));
  chassis->mutable_ch()->mutable_ecu_status_3_517()->set_ultrasound_dist_6(
      ultrasound_dist_6(bytes, length));
  chassis->mutable_ch()->mutable_ecu_status_3_517()->set_ultrasound_dist_7(
      ultrasound_dist_7(bytes, length));
  chassis->mutable_ch()->mutable_ecu_status_3_517()->set_ultrasound_dist_8(
      ultrasound_dist_8(bytes, length));
}

// config detail: {'description': 'Ultrasonic detection distance 1 (Ultrasound
// status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
// 'ultrasound_dist_1', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
int Ecustatus3517::ultrasound_dist_1(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Ultrasonic detection distance 2 (Ultrasound
// status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
// 'ultrasound_dist_2', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
int Ecustatus3517::ultrasound_dist_2(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Ultrasonic detection distance 3 (Ultrasound
// status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
// 'ultrasound_dist_3', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
int Ecustatus3517::ultrasound_dist_3(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Ultrasonic detection distance 4 (Ultrasound
// status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
// 'ultrasound_dist_4', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
int Ecustatus3517::ultrasound_dist_4(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Ultrasonic detection distance 5 (Ultrasound
// status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
// 'ultrasound_dist_5', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
int Ecustatus3517::ultrasound_dist_5(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Ultrasonic detection distance 6 (Ultrasound
// status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
// 'ultrasound_dist_6', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
int Ecustatus3517::ultrasound_dist_6(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Ultrasonic detection distance 7 (Ultrasound
// status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
// 'ultrasound_dist_7', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
int Ecustatus3517::ultrasound_dist_7(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Ultrasonic detection distance 8 (Ultrasound
// status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
// 'ultrasound_dist_8', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
int Ecustatus3517::ultrasound_dist_8(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
