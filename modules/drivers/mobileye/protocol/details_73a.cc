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

#include "modules/drivers/mobileye/protocol/details_73a.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace mobileye {

using ::apollo::drivers::canbus::Byte;

const int Details73a::ID = 0x73A;

void Details73a::Parse(const uint8_t* bytes, int32_t length,
                       Mobileye* mobileye) const {
  auto* details_73a = mobileye->add_details_73a();
  details_73a->set_obstacle_length(obstacle_length(bytes, length));
  details_73a->set_obstacle_width(obstacle_width(bytes, length));
  details_73a->set_obstacle_age(obstacle_age(bytes, length));
  details_73a->set_obstacle_lane(obstacle_lane(bytes, length));
  details_73a->set_cipv_flag(is_cipv_flag(bytes, length));
  details_73a->set_reserved_5(is_reserved_5(bytes, length));
  details_73a->set_radar_pos_x(radar_pos_x(bytes, length));
  details_73a->set_radar_vel_x(radar_vel_x(bytes, length));
  details_73a->set_radar_match_confidence(
      radar_match_confidence(bytes, length));
  details_73a->set_reserved_6(is_reserved_6(bytes, length));
  details_73a->set_matched_radar_id(matched_radar_id(bytes, length));
  details_73a->set_reserved_7(is_reserved_7(bytes, length));
}

// config detail: {'name': 'obstacle_length', 'offset': 0.0, 'precision': 0.5,
// 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|31]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit':
// '"meters"'}
double Details73a::obstacle_length(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  return x * 0.500000;
}

// config detail: {'name': 'obstacle_width', 'offset': 0.0, 'precision': 0.05,
// 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|12.75]', 'bit': 8, 'type': 'double', 'order': 'intel', 'physical_unit':
// '"meters"'}
double Details73a::obstacle_width(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  return x * 0.050000;
}

// config detail: {'name': 'obstacle_age', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details73a::obstacle_age(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  return x;
}

// config detail: {'name': 'obstacle_lane', 'offset': 0.0, 'precision': 1.0,
// 'len': 2, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|3]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details73a::obstacle_lane(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 2);

  return x;
}

// config detail: {'name': 'cipv_flag', 'offset': 0.0, 'precision': 1.0, 'len':
// 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 26, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool Details73a::is_cipv_flag(const uint8_t* bytes, int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(2);
}

// config detail: {'name': 'reserved_5', 'offset': 0.0, 'precision': 1.0, 'len':
// 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 27, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool Details73a::is_reserved_5(const uint8_t* bytes, int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(3);
}

// config detail: {'name': 'radar_pos_x', 'offset': 0.0, 'precision': 0.0625,
// 'len': 12, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|255.9375]', 'bit': 28, 'type': 'double', 'order': 'intel',
// 'physical_unit': '"meters"'}
double Details73a::radar_pos_x(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  return x * 0.062500;
}

// config detail: {'name': 'radar_vel_x', 'offset': 0.0, 'precision': 0.0625,
// 'len': 12, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
// '[-128|127.9375]', 'bit': 40, 'type': 'double', 'order': 'intel',
// 'physical_unit': '"meters/sec"'}
double Details73a::radar_vel_x(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 20;
  x >>= 20;

  return x * 0.062500;
}

// config detail: {'name': 'radar_match_confidence', 'offset': 0.0, 'precision':
// 1.0, 'len': 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|7]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details73a::radar_match_confidence(const uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 3);

  return x;
}

// config detail: {'name': 'reserved_6', 'offset': 0.0, 'precision': 1.0, 'len':
// 1, 'f_type': 'valid', 'is_signed_var': True, 'physical_range': '[0|0]',
// 'bit': 55, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool Details73a::is_reserved_6(const uint8_t* bytes, int32_t length) const {
  Byte frame(bytes + 6);
  return frame.is_bit_1(7);
}

// config detail: {'name': 'matched_radar_id', 'offset': 0.0, 'precision': 1.0,
// 'len': 7, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|127]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details73a::matched_radar_id(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 7);

  return x;
}

// config detail: {'name': 'reserved_7', 'offset': 0.0, 'precision': 1.0, 'len':
// 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 63, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool Details73a::is_reserved_7(const uint8_t* bytes, int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(7);
}

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo
