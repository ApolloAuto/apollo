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

#include "modules/drivers/mobileye/protocol/details_738.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace mobileye {

using ::apollo::drivers::canbus::Byte;

const int Details738::ID = 0x738;

void Details738::Parse(const uint8_t *bytes, int32_t length,
                       Mobileye *mobileye) const {
  mobileye->mutable_details_738()->set_num_obstacles(
      num_obstacles(bytes, length));
  mobileye->mutable_details_738()->set_timestamp(timestamp(bytes, length));
  mobileye->mutable_details_738()->set_application_version(
      application_version(bytes, length));
  mobileye->mutable_details_738()->set_active_version_number_section(
      active_version_number_section(bytes, length));
  mobileye->mutable_details_738()->set_left_close_rang_cut_in(
      is_left_close_rang_cut_in(bytes, length));
  mobileye->mutable_details_738()->set_right_close_rang_cut_in(
      is_right_close_rang_cut_in(bytes, length));
  mobileye->mutable_details_738()->set_go(go(bytes, length));
  mobileye->mutable_details_738()->set_protocol_version(
      protocol_version(bytes, length));
  mobileye->mutable_details_738()->set_close_car(is_close_car(bytes, length));
  mobileye->mutable_details_738()->set_failsafe(failsafe(bytes, length));
  mobileye->mutable_details_738()->set_reserved_10(reserved_10(bytes, length));
  mobileye->mutable_details_739()->Clear();
  mobileye->mutable_details_73a()->Clear();
  mobileye->mutable_details_73b()->Clear();
}

// config detail: {'name': 'num_obstacles', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details738::num_obstacles(const uint8_t *bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  return x;
}

// config detail: {'name': 'timestamp', 'offset': 0.0, 'precision': 1.0, 'len':
// 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|255]',
// 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': '"ms"'}
int Details738::timestamp(const uint8_t *bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  return x;
}

// config detail: {'name': 'application_version', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details738::application_version(const uint8_t *bytes,
                                    int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  return x;
}

// config detail: {'name': 'active_version_number_section', 'offset': 0.0,
// 'precision': 1.0, 'len': 2, 'f_type': 'value', 'is_signed_var': False,
// 'physical_range': '[0|3]', 'bit': 24, 'type': 'int', 'order': 'intel',
// 'physical_unit': '""'}
int Details738::active_version_number_section(const uint8_t *bytes,
                                              int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 2);

  return x;
}

// config detail: {'name': 'left_close_rang_cut_in', 'offset': 0.0, 'precision':
// 1.0, 'len': 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 26, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool Details738::is_left_close_rang_cut_in(const uint8_t *bytes,
                                           int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(2);
}

// config detail: {'name': 'right_close_rang_cut_in', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'f_type': 'valid', 'is_signed_var': False,
// 'physical_range': '[0|0]', 'bit': 27, 'type': 'bool', 'order': 'intel',
// 'physical_unit': '""'}
bool Details738::is_right_close_rang_cut_in(const uint8_t *bytes,
                                            int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(3);
}

// config detail: {'name': 'go', 'offset': 0.0, 'precision': 1.0, 'len': 4,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 28, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details738::go(const uint8_t *bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 4);

  return x;
}

// config detail: {'name': 'protocol_version', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details738::protocol_version(const uint8_t *bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  return x;
}

// config detail: {'name': 'close_car', 'offset': 0.0, 'precision': 1.0, 'len':
// 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 40, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool Details738::is_close_car(const uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 5);
  return frame.is_bit_1(0);
}

// config detail: {'name': 'failsafe', 'offset': 0.0, 'precision': 1.0, 'len':
// 4, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 41, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details738::failsafe(const uint8_t *bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(1, 4);

  return x;
}

// config detail: {'name': 'reserved_10', 'offset': 0.0, 'precision': 1.0,
// 'len': 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 45, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Details738::reserved_10(const uint8_t *bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(5, 3);

  return x;
}

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo
