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

#include "modules/drivers/mobileye/protocol/details_739.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace mobileye {

using ::apollo::drivers::canbus::Byte;

const int Details739::ID = 0x739;

void Details739::Parse(const uint8_t* bytes, int32_t length,
                       Mobileye* mobileye) const {
  auto* details_739 = mobileye->add_details_739();
  details_739->set_obstacle_id(obstacle_id(bytes, length));
  details_739->set_obstacle_pos_x(obstacle_pos_x(bytes, length));
  details_739->set_reseved_2(reseved_2(bytes, length));
  details_739->set_obstacle_pos_y(obstacle_pos_y(bytes, length));
  details_739->set_blinker_info(blinker_info(bytes, length));
  details_739->set_cut_in_and_out(cut_in_and_out(bytes, length));
  details_739->set_obstacle_rel_vel_x(obstacle_rel_vel_x(bytes, length));
  details_739->set_obstacle_type(obstacle_type(bytes, length));
  details_739->set_reserved_3(is_reserved_3(bytes, length));
  details_739->set_obstacle_status(obstacle_status(bytes, length));
  details_739->set_obstacle_brake_lights(
      is_obstacle_brake_lights(bytes, length));
  details_739->set_reserved_4(reserved_4(bytes, length));
  details_739->set_obstacle_valid(obstacle_valid(bytes, length));
}

// config detail: {'name': 'obstacle_id', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|63]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details739::obstacle_id(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  return x;
}

// config detail: {'name': 'obstacle_pos_x', 'offset': 0.0, 'precision': 0.0625,
// 'len': 12, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|255.9375]', 'bit': 8, 'type': 'double', 'order': 'intel',
// 'physical_unit': '"meters"'}
double Details739::obstacle_pos_x(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  return x * 0.062500;
}

// config detail: {'name': 'reseved_2', 'offset': 0.0, 'precision': 1.0, 'len':
// 4, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 20, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details739::reseved_2(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 4);

  return x;
}

// config detail: {'name': 'obstacle_pos_y', 'offset': 0.0, 'precision': 0.0625,
// 'len': 10, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
// '[-32|31.9375]', 'bit': 24, 'type': 'double', 'order': 'intel',
// 'physical_unit': '"meters"'}
double Details739::obstacle_pos_y(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 2);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 22;
  x >>= 22;

  return x * 0.062500;
}

// config detail: {'name': 'blinker_info', 'offset': 0.0, 'precision': 1.0,
// 'len': 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|7]', 'bit': 34, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details739::blinker_info(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 3);

  return x;
}

// config detail: {'name': 'cut_in_and_out', 'offset': 0.0, 'precision': 1.0,
// 'len': 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|7]', 'bit': 37, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details739::cut_in_and_out(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(5, 3);

  return x;
}

// config detail: {'name': 'obstacle_rel_vel_x', 'offset': 0.0, 'precision':
// 0.0625, 'len': 12, 'f_type': 'value', 'is_signed_var': True,
// 'physical_range': '[-128|127.9375]', 'bit': 40, 'type': 'double', 'order':
// 'intel', 'physical_unit': '"meters/sec"'}
double Details739::obstacle_rel_vel_x(const uint8_t* bytes,
                                      int32_t length) const {
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

// config detail: {'name': 'obstacle_type', 'offset': 0.0, 'precision': 1.0,
// 'len': 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|7]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details739::obstacle_type(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 3);

  return x;
}

// config detail: {'name': 'reserved_3', 'offset': 0.0, 'precision': 1.0, 'len':
// 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 55, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool Details739::is_reserved_3(const uint8_t* bytes, int32_t length) const {
  Byte frame(bytes + 6);
  return frame.is_bit_1(7);
}

// config detail: {'name': 'obstacle_status', 'offset': 0.0, 'precision': 1.0,
// 'len': 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|7]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details739::obstacle_status(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 3);

  return x;
}

// config detail: {'name': 'obstacle_brake_lights', 'offset': 0.0, 'precision':
// 1.0, 'len': 1, 'f_type': 'valid', 'is_signed_var': True, 'physical_range':
// '[0|0]', 'bit': 59, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool Details739::is_obstacle_brake_lights(const uint8_t* bytes,
                                          int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(3);
}

// config detail: {'name': 'reserved_4', 'offset': 0.0, 'precision': 1.0, 'len':
// 2, 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[0|0]',
// 'bit': 60, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details739::reserved_4(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(4, 2);

  x <<= 30;
  x >>= 30;

  return x;
}

// config detail: {'name': 'obstacle_valid', 'offset': 0.0, 'precision': 1.0,
// 'len': 2, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|3]', 'bit': 62, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details739::obstacle_valid(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(6, 2);

  return x;
}

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo
