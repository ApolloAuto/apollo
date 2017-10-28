/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/mobileye/protocol/aftermarket_669.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace mobileye {

using ::apollo::drivers::canbus::Byte;

const int Aftermarket669::ID = 0x669;

void Aftermarket669::Parse(const uint8_t* bytes, int32_t length,
                           Mobileye* mobileye) const {
  mobileye->mutable_aftermarket_669()->set_lane_conf_left(
      lane_conf_left(bytes, length));
  mobileye->mutable_aftermarket_669()->set_ldw_availability_left(
      is_ldw_availability_left(bytes, length));
  mobileye->mutable_aftermarket_669()->set_lane_type_left(
      lane_type_left(bytes, length));
  mobileye->mutable_aftermarket_669()->set_distance_to_lane_l(
      distance_to_lane_l(bytes, length));
  mobileye->mutable_aftermarket_669()->set_lane_conf_right(
      lane_conf_right(bytes, length));
  mobileye->mutable_aftermarket_669()->set_ldw_availability_right(
      is_ldw_availability_right(bytes, length));
  mobileye->mutable_aftermarket_669()->set_lane_type_right(
      lane_type_right(bytes, length));
  mobileye->mutable_aftermarket_669()->set_distance_to_lane_r(
      distance_to_lane_r(bytes, length));
}

// config detail: {'name': 'lane_conf_left', 'offset': 0.0, 'precision': 1.0,
// 'len': 2, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
// '[0|3]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Aftermarket669::lane_conf_left(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  x <<= 30;
  x >>= 30;

  return x;
}

// config detail: {'name': 'ldw_availability_left', 'offset': 0.0, 'precision':
// 1.0, 'len': 1, 'f_type': 'valid', 'is_signed_var': True, 'physical_range':
// '[0|1]', 'bit': 2, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool Aftermarket669::is_ldw_availability_left(const uint8_t* bytes,
                                              int32_t length) const {
  Byte frame(bytes + 0);
  return frame.is_bit_1(2);
}

// config detail: {'name': 'lane_type_left', 'offset': 0.0, 'precision': 1.0,
// 'len': 4, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
// '[0|6]', 'bit': 4, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Aftermarket669::lane_type_left(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 4);

  x <<= 28;
  x >>= 28;

  return x;
}

// config detail: {'name': 'distance_to_lane_l', 'offset': 0.0, 'precision':
// 0.02, 'len': 12, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
// '[-40|40]', 'bit': 12, 'type': 'double', 'order': 'intel', 'physical_unit':
// '"meters"'}
double Aftermarket669::distance_to_lane_l(const uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 20;
  x >>= 20;

  return x * 0.020000;
}

// config detail: {'name': 'lane_conf_right', 'offset': 0.0, 'precision': 1.0,
// 'len': 2, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
// '[0|3]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Aftermarket669::lane_conf_right(const uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 2);

  x <<= 30;
  x >>= 30;

  return x;
}

// config detail: {'name': 'ldw_availability_right', 'offset': 0.0, 'precision':
// 1.0, 'len': 1, 'f_type': 'valid', 'is_signed_var': True, 'physical_range':
// '[0|1]', 'bit': 42, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool Aftermarket669::is_ldw_availability_right(const uint8_t* bytes,
                                               int32_t length) const {
  Byte frame(bytes + 5);
  return frame.is_bit_1(2);
}

// config detail: {'name': 'lane_type_right', 'offset': 0.0, 'precision': 1.0,
// 'len': 4, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
// '[0|6]', 'bit': 44, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Aftermarket669::lane_type_right(const uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 4);

  x <<= 28;
  x >>= 28;

  return x;
}

// config detail: {'name': 'distance_to_lane_r', 'offset': 0.0, 'precision':
// 0.02, 'len': 12, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
// '[-40|40]', 'bit': 52, 'type': 'double', 'order': 'intel', 'physical_unit':
// '"meters"'}
double Aftermarket669::distance_to_lane_r(const uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 20;
  x >>= 20;

  return x * 0.020000;
}

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo
