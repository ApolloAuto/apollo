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

#include "modules/drivers/mobileye/protocol/lka_766.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace mobileye {

using ::apollo::drivers::canbus::Byte;

const int Lka766::ID = 0x766;

void Lka766::Parse(const uint8_t* bytes, int32_t length,
                   Mobileye* mobileye) const {
  mobileye->mutable_lka_766()->set_lane_type(lane_type(bytes, length));
  mobileye->mutable_lka_766()->set_quality(quality(bytes, length));
  mobileye->mutable_lka_766()->set_model_degree(model_degree(bytes, length));
  mobileye->mutable_lka_766()->set_position(position(bytes, length));
  mobileye->mutable_lka_766()->set_curvature(curvature(bytes, length));
  mobileye->mutable_lka_766()->set_curvature_derivative(
      curvature_derivative(bytes, length));
  mobileye->mutable_lka_766()->set_width_left_marking(
      width_left_marking(bytes, length));
}

// config detail: {'name': 'lane_type', 'offset': 0.0, 'precision': 1.0, 'len':
// 4, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
// 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Lka766::lane_type(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 4);

  return x;
}

// config detail: {'name': 'quality', 'offset': 0.0, 'precision': 1.0, 'len': 2,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 4, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Lka766::quality(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 2);

  return x;
}

// config detail: {'name': 'model_degree', 'offset': 0.0, 'precision': 1.0,
// 'len': 2, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 6, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Lka766::model_degree(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  return x;
}

// config detail: {'name': 'position', 'offset': 0.0, 'precision': 0.00390625,
// 'len': 16, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
// '[-128|127]', 'bit': 8, 'type': 'double', 'order': 'intel', 'physical_unit':
// '"meter"'}
double Lka766::position(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  return x * 0.00390625;
}

// config detail: {'name': 'curvature', 'offset': -0.031999023438, 'precision':
// 9.765625e-07, 'len': 16, 'f_type': 'value', 'is_signed_var': False,
// 'physical_range': '[-0.02|0.02]', 'bit': 24, 'type': 'double', 'order':
// 'intel', 'physical_unit': '"1/meter"'}
double Lka766::curvature(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  return x * 9.765625e-07 + -0.031999023438;
}

// config detail: {'name': 'curvature_derivative', 'offset': -0.00012206658721,
// 'precision': 3.7252902985e-09, 'len': 16, 'f_type': 'value', 'is_signed_var':
// False, 'physical_range': '[-0.00012|0.00012]', 'bit': 40, 'type': 'double',
// 'order': 'intel', 'physical_unit': '"1/meter^2"'}
double Lka766::curvature_derivative(const uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  return x * 3.7252902985e-09 + -0.00012206658721;
}

// config detail: {'name': 'width_left_marking', 'offset': 0.0, 'precision':
// 0.01, 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|2.5]', 'bit': 56, 'type': 'double', 'order': 'intel', 'physical_unit':
// '"m"'}
double Lka766::width_left_marking(const uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  return x * 0.010000;
}

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo
