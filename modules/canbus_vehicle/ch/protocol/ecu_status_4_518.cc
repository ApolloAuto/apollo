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

#include "modules/canbus_vehicle/ch/protocol/ecu_status_4_518.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Ecustatus4518::Ecustatus4518() {}
const int32_t Ecustatus4518::ID = 0x518;

void Ecustatus4518::Parse(const std::uint8_t* bytes, int32_t length,
                          Ch* chassis) const {
  chassis->mutable_ecu_status_4_518()->set_ultrasound_dist_9(
      ultrasound_dist_9(bytes, length));
  chassis->mutable_ecu_status_4_518()->set_ultrasound_dist_10(
      ultrasound_dist_10(bytes, length));
  chassis->mutable_ecu_status_4_518()->set_ultrasound_dist_11(
      ultrasound_dist_11(bytes, length));
  chassis->mutable_ecu_status_4_518()->set_ultrasound_dist_12(
      ultrasound_dist_12(bytes, length));
  chassis->mutable_ecu_status_4_518()->set_ultrasound_dist_13(
      ultrasound_dist_13(bytes, length));
  chassis->mutable_ecu_status_4_518()->set_ultrasound_dist_14(
      ultrasound_dist_14(bytes, length));
  chassis->mutable_ecu_status_4_518()->set_ultrasound_dist_15(
      ultrasound_dist_15(bytes, length));
  chassis->mutable_ecu_status_4_518()->set_ultrasound_dist_16(
      ultrasound_dist_16(bytes, length));
}

// config detail: {'bit': 0, 'description': 'Ultrasonic detection distance 9
// (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
// 'ultrasound_dist_9', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
double Ecustatus4518::ultrasound_dist_9(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'bit': 8, 'description': 'Ultrasonic detection distance 10
// (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
// 'ultrasound_dist_10', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
double Ecustatus4518::ultrasound_dist_10(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'bit': 16, 'description': 'Ultrasonic detection distance 11
// (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
// 'ultrasound_dist_11', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
double Ecustatus4518::ultrasound_dist_11(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'bit': 24, 'description': 'Ultrasonic detection distance 12
// (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
// 'ultrasound_dist_12', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
double Ecustatus4518::ultrasound_dist_12(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'bit': 32, 'description': 'Ultrasonic detection distance 13
// (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
// 'ultrasound_dist_13', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
double Ecustatus4518::ultrasound_dist_13(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'bit': 40, 'description': 'Ultrasonic detection distance 14
// (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
// 'ultrasound_dist_14', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
double Ecustatus4518::ultrasound_dist_14(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'bit': 48, 'description': 'Ultrasonic detection distance 15
// (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
// 'ultrasound_dist_15', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
double Ecustatus4518::ultrasound_dist_15(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'bit': 56, 'description': 'Ultrasonic detection distance 16
// (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
// 'ultrasound_dist_16', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
double Ecustatus4518::ultrasound_dist_16(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
