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

#include "modules/drivers/mobileye/protocol/reference_76a.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace mobileye {

using ::apollo::drivers::canbus::Byte;

const int Reference76a::ID = 0x76a;

void Reference76a::Parse(const uint8_t* bytes, int32_t length,
                         Mobileye* mobileye) const {
  mobileye->mutable_reference_76a()->set_ref_point_1_position(
      ref_point_1_position(bytes, length));
  mobileye->mutable_reference_76a()->set_ref_point_1_distance(
      ref_point_1_distance(bytes, length));
  mobileye->mutable_reference_76a()->set_ref_point_1_validity(
      is_ref_point_1_validity(bytes, length));
  mobileye->mutable_reference_76a()->set_ref_point_2_position(
      ref_point_2_position(bytes, length));
  mobileye->mutable_reference_76a()->set_ref_point_2_distance(
      ref_point_2_distance(bytes, length));
  mobileye->mutable_reference_76a()->set_ref_point_2_validity(
      is_ref_point_2_validity(bytes, length));
}

// config detail: {'name': 'ref_point_1_position', 'offset': -127.99609375,
// 'precision': 0.00390625, 'len': 16, 'f_type': 'value', 'is_signed_var':
// False, 'physical_range': '[-127.99609375|128]', 'bit': 0, 'type': 'double',
// 'order': 'intel', 'physical_unit': '"meters"'}
double Reference76a::ref_point_1_position(const uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  return x * 0.003906 + -127.996094;
}

// config detail: {'name': 'ref_point_1_distance', 'offset': 0.0, 'precision':
// 0.00390625, 'len': 15, 'f_type': 'value', 'is_signed_var': False,
// 'physical_range': '[0|127.99609375]', 'bit': 16, 'type': 'double', 'order':
// 'intel', 'physical_unit': '"meters"'}
double Reference76a::ref_point_1_distance(const uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 7);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  return x * 0.003906;
}

// config detail: {'name': 'ref_point_1_validity', 'offset': 0.0, 'precision':
// 1.0, 'len': 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 31, 'type': 'bool', 'order': 'motorola', 'physical_unit':
// '""'}
bool Reference76a::is_ref_point_1_validity(const uint8_t* bytes,
                                           int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(7);
}

// config detail: {'name': 'ref_point_2_position', 'offset': -127.99609375,
// 'precision': 0.00390625, 'len': 16, 'f_type': 'value', 'is_signed_var':
// False, 'physical_range': '[-127.99609375|128]', 'bit': 32, 'type': 'double',
// 'order': 'intel', 'physical_unit': '"meters"'}
double Reference76a::ref_point_2_position(const uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  return x * 0.003906 + -127.996094;
}

// config detail: {'name': 'ref_point_2_distance', 'offset': 0.0, 'precision':
// 0.00390625, 'len': 15, 'f_type': 'value', 'is_signed_var': False,
// 'physical_range': '[0|127.99609375]', 'bit': 48, 'type': 'double', 'order':
// 'intel', 'physical_unit': '"meters"'}
double Reference76a::ref_point_2_distance(const uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 7);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  return x * 0.003906;
}

// config detail: {'name': 'ref_point_2_validity', 'offset': 0.0, 'precision':
// 1.0, 'len': 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 63, 'type': 'bool', 'order': 'motorola', 'physical_unit':
// '""'}
bool Reference76a::is_ref_point_2_validity(const uint8_t* bytes,
                                           int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(7);
}

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo
