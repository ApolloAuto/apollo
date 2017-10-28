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

#include "modules/drivers/mobileye/protocol/details_737.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace mobileye {

using ::apollo::drivers::canbus::Byte;

const int Details737::ID = 0x737;

void Details737::Parse(const uint8_t* bytes, int32_t length,
                       Mobileye* mobileye) const {
  mobileye->mutable_details_737()->set_lane_curvature(
      lane_curvature(bytes, length));
  mobileye->mutable_details_737()->set_lane_heading(
      lane_heading(bytes, length));
  mobileye->mutable_details_737()->set_ca_construction_area(
      is_ca_construction_area(bytes, length));
  mobileye->mutable_details_737()->set_right_ldw_availability(
      is_right_ldw_availability(bytes, length));
  mobileye->mutable_details_737()->set_left_ldw_availability(
      is_left_ldw_availability(bytes, length));
  mobileye->mutable_details_737()->set_reserved_1(is_reserved_1(bytes, length));
  mobileye->mutable_details_737()->set_yaw_angle(yaw_angle(bytes, length));
  mobileye->mutable_details_737()->set_pitch_angle(pitch_angle(bytes, length));
}

// config detail: {'name': 'lane_curvature', 'offset': 0.0, 'precision':
// 3.81e-06, 'len': 16, 'f_type': 'value', 'is_signed_var': True,
// 'physical_range': '[-0.12484608|0.12484227]', 'bit': 0, 'type': 'double',
// 'order': 'intel', 'physical_unit': '"1/meters"'}
double Details737::lane_curvature(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  return x * 0.000004;
}

// config detail: {'name': 'lane_heading', 'offset': 0.0, 'precision': 0.0005,
// 'len': 12, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
// '[-1|1]', 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit':
// '""'}
double Details737::lane_heading(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 20;
  x >>= 20;

  return x * 0.000500;
}

// config detail: {'name': 'ca_construction_area', 'offset': 0.0, 'precision':
// 1.0, 'len': 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 28, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool Details737::is_ca_construction_area(const uint8_t* bytes,
                                         int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(4);
}

// config detail: {'name': 'right_ldw_availability', 'offset': 0.0, 'precision':
// 1.0, 'len': 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 29, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool Details737::is_right_ldw_availability(const uint8_t* bytes,
                                           int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(5);
}

// config detail: {'name': 'left_ldw_availability', 'offset': 0.0, 'precision':
// 1.0, 'len': 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 30, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool Details737::is_left_ldw_availability(const uint8_t* bytes,
                                          int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(6);
}

// config detail: {'name': 'reserved_1', 'offset': 0.0, 'precision': 1.0, 'len':
// 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 31, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool Details737::is_reserved_1(const uint8_t* bytes, int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(7);
}

// config detail: {'name': 'yaw_angle', 'offset': 0.0, 'precision':
// 0.0009765625, 'len': 16, 'f_type': 'value', 'is_signed_var': True,
// 'physical_range': '[-32|31.999023438]', 'bit': 32, 'type': 'double', 'order':
// 'intel', 'physical_unit': '"radians"'}
double Details737::yaw_angle(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  return x * 0.0009765625;
}

// config detail: {'name': 'pitch_angle', 'offset': 0.0, 'precision':
// 1.90734e-06, 'len': 16, 'f_type': 'value', 'is_signed_var': True,
// 'physical_range': '[-0.06249971712|0.06249780978]', 'bit': 48, 'type':
// 'double', 'order': 'intel', 'physical_unit': '"radians"'}
double Details737::pitch_angle(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  return x * 1.90734e-06;
}

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo
