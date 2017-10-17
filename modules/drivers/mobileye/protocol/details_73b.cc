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

#include "modules/drivers/mobileye/protocol/details_73b.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace mobileye {

using ::apollo::drivers::canbus::Byte;

const int Details73b::ID = 0x73B;

void Details73b::Parse(const uint8_t* bytes, int32_t length,
                       Mobileye* mobileye) const {
  auto* details_73b = mobileye->add_details_73b();
  details_73b->set_obstacle_angle_rate(obstacle_angle_rate(bytes, length));
  details_73b->set_obstacle_scale_change(obstacle_scale_change(bytes, length));
  details_73b->set_object_accel_x(object_accel_x(bytes, length));
  details_73b->set_reserved_8(reserved_8(bytes, length));
  details_73b->set_obstacle_replaced(is_obstacle_replaced(bytes, length));
  details_73b->set_reserved_9(reserved_9(bytes, length));
  details_73b->set_obstacle_angle(obstacle_angle(bytes, length));
}

// config detail: {'name': 'obstacle_angle_rate', 'offset': 0.0, 'precision':
// 0.01, 'len': 16, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
// '[-327.68|327.67]', 'bit': 0, 'type': 'double', 'order': 'intel',
// 'physical_unit': '"degree"'}
double Details73b::obstacle_angle_rate(const uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  return x * 0.010000;
}

// config detail: {'name': 'obstacle_scale_change', 'offset': 0.0, 'precision':
// 0.0002, 'len': 16, 'f_type': 'value', 'is_signed_var': True,
// 'physical_range': '[-6.5536|6.5534]', 'bit': 16, 'type': 'double', 'order':
// 'intel', 'physical_unit': '"pix/sec"'}
double Details73b::obstacle_scale_change(const uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  return x * 0.000200;
}

// config detail: {'name': 'object_accel_x', 'offset': 0.0, 'precision': 0.03,
// 'len': 10, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
// '[-15.36|15.33]', 'bit': 32, 'type': 'double', 'order': 'intel',
// 'physical_unit': '"m/S2"'}
double Details73b::object_accel_x(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 2);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 22;
  x >>= 22;

  return x * 0.030000;
}

// config detail: {'name': 'reserved_8', 'offset': 0.0, 'precision': 1.0, 'len':
// 2, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 42, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details73b::reserved_8(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(2, 2);

  return x;
}

// config detail: {'name': 'obstacle_replaced', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 44, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool Details73b::is_obstacle_replaced(const uint8_t* bytes,
                                      int32_t length) const {
  Byte frame(bytes + 5);
  return frame.is_bit_1(4);
}

// config detail: {'name': 'reserved_9', 'offset': 0.0, 'precision': 1.0, 'len':
// 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 45, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details73b::reserved_9(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(5, 3);

  return x;
}

// config detail: {'name': 'obstacle_angle', 'offset': 0.0, 'precision': 0.01,
// 'len': 16, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
// '[-327.68|327.67]', 'bit': 48, 'type': 'double', 'order': 'intel',
// 'physical_unit': '"degree"'}
double Details73b::obstacle_angle(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  return x * 0.010000;
}

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo
