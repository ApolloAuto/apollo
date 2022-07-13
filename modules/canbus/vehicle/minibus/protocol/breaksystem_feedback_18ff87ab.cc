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

#include "modules/canbus/vehicle/minibus/protocol/breaksystem_feedback_18ff87ab.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

Breaksystemfeedback18ff87ab::Breaksystemfeedback18ff87ab() {}
const int32_t Breaksystemfeedback18ff87ab::ID = 0x38ff87ab;

void Breaksystemfeedback18ff87ab::Parse(const std::uint8_t* bytes,
                                        int32_t length,
                                        ChassisDetail* chassis) const {
  chassis->mutable_minibus()
      ->mutable_breaksystem_feedback_18ff87ab()
      ->set_brk_fb_systemdefault(brk_fb_systemdefault(bytes, length));
  chassis->mutable_minibus()
      ->mutable_breaksystem_feedback_18ff87ab()
      ->set_epb_fb_main_brkpressure(epb_fb_main_brkpressure(bytes, length));
  chassis->mutable_minibus()
      ->mutable_breaksystem_feedback_18ff87ab()
      ->set_epb_fb_right_brkpressure_setvaul(
          epb_fb_right_brkpressure_setvaul(bytes, length));
  chassis->mutable_minibus()
      ->mutable_breaksystem_feedback_18ff87ab()
      ->set_epb_fb_left_brkpressure_setvaule(
          epb_fb_left_brkpressure_setvaule(bytes, length));
  chassis->mutable_minibus()
      ->mutable_breaksystem_feedback_18ff87ab()
      ->set_brk_fb_epb_feedback(brk_fb_epb_feedback(bytes, length));
  chassis->mutable_minibus()
      ->mutable_breaksystem_feedback_18ff87ab()
      ->set_bek_fb_break_enable(bek_fb_break_enable(bytes, length));
  chassis->mutable_minibus()
      ->mutable_breaksystem_feedback_18ff87ab()
      ->set_brk_fb_overhot_warning(brk_fb_overhot_warning(bytes, length));
  chassis->mutable_minibus()
      ->mutable_breaksystem_feedback_18ff87ab()
      ->set_brk_fb_right_breakpressure(
          brk_fb_right_breakpressure(bytes, length));
  chassis->mutable_minibus()
      ->mutable_breaksystem_feedback_18ff87ab()
      ->set_brk_fb_left_breakpressure(brk_fb_left_breakpressure(bytes, length));
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
// 'brk_fb_systemdefault', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|25.5]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double Breaksystemfeedback18ff87ab::brk_fb_systemdefault(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
// 'epb_fb_main_brkpressure', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double Breaksystemfeedback18ff87ab::epb_fb_main_brkpressure(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name':
// 'epb_fb_right_brkpressure_setvaul', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.1, 'type':
// 'double'}
double Breaksystemfeedback18ff87ab::epb_fb_right_brkpressure_setvaul(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name':
// 'epb_fb_left_brkpressure_setvaule', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.1, 'type':
// 'double'}
double Breaksystemfeedback18ff87ab::epb_fb_left_brkpressure_setvaule(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 31, 'is_signed_var': False, 'len': 1, 'name':
// 'brk_fb_epb_feedback', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Breaksystemfeedback18ff87ab::brk_fb_epb_feedback(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 30, 'is_signed_var': False, 'len': 1, 'name':
// 'bek_fb_break_enable', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Breaksystemfeedback18ff87ab::bek_fb_break_enable(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 29, 'is_signed_var': False, 'len': 1, 'name':
// 'brk_fb_overhot_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Breaksystemfeedback18ff87ab::brk_fb_overhot_warning(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': True, 'len': 8, 'name':
// 'brk_fb_right_breakpressure', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[-12.8|12.7]', 'physical_unit': '', 'precision': 0.1,
// 'type': 'double'}
double Breaksystemfeedback18ff87ab::brk_fb_right_breakpressure(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name':
// 'brk_fb_left_breakpressure', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|25.5]', 'physical_unit': '', 'precision': 0.1, 'type':
// 'double'}
double Breaksystemfeedback18ff87ab::brk_fb_left_breakpressure(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.100000;
  return ret;
}
}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
