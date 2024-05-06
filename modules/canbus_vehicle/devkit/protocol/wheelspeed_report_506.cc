/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/devkit/protocol/wheelspeed_report_506.h"

#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::drivers::canbus::Byte;

Wheelspeedreport506::Wheelspeedreport506() {}
const int32_t Wheelspeedreport506::ID = 0x506;

void Wheelspeedreport506::Parse(const std::uint8_t* bytes, int32_t length,
                                Devkit* chassis) const {
  chassis->mutable_wheelspeed_report_506()->set_rr(
      rr(bytes, length));
  chassis->mutable_wheelspeed_report_506()->set_rl(
      rl(bytes, length));
  chassis->mutable_wheelspeed_report_506()->set_fr(
      fr(bytes, length));
  chassis->mutable_wheelspeed_report_506()->set_fl(
      fl(bytes, length));
}

// config detail: {'name': 'rr', 'offset': 0.0, 'precision': 0.001, 'len': 16,
// 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit': 55, 'type':
// 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
double Wheelspeedreport506::rr(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'rl', 'offset': 0.0, 'precision': 0.001, 'len': 16,
// 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit': 39, 'type':
// 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
double Wheelspeedreport506::rl(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'fr', 'offset': 0.0, 'precision': 0.001, 'len': 16,
// 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit': 23, 'type':
// 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
double Wheelspeedreport506::fr(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'fl', 'offset': 0.0, 'precision': 0.001, 'len': 16,
// 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit': 7, 'type':
// 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
double Wheelspeedreport506::fl(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}
}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
