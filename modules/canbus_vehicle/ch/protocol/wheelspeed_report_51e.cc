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

#include "modules/canbus_vehicle/ch/protocol/wheelspeed_report_51e.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Wheelspeedreport51e::Wheelspeedreport51e() {}
const int32_t Wheelspeedreport51e::ID = 0x51E;

void Wheelspeedreport51e::Parse(const std::uint8_t* bytes, int32_t length,
                                Ch* chassis) const {
  chassis->mutable_wheelspeed_report_51e()->set_rr(
      rr(bytes, length));
  chassis->mutable_wheelspeed_report_51e()->set_rl(
      rl(bytes, length));
  chassis->mutable_wheelspeed_report_51e()->set_fr(
      fr(bytes, length));
  chassis->mutable_wheelspeed_report_51e()->set_fl(
      fl(bytes, length));
}

// config detail: {'bit': 48, 'description': 'wheel speed rear right',
// 'is_signed_var': True, 'len': 16, 'name': 'rr', 'offset': 0.0, 'order':
// 'intel', 'physical_range': '[-327.68|327.67]', 'physical_unit': 'm/s',
// 'precision': 0.01, 'type': 'double'}
double Wheelspeedreport51e::rr(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 32, 'description': 'wheel speed rear left',
// 'is_signed_var': True, 'len': 16, 'name': 'rl', 'offset': 0.0, 'order':
// 'intel', 'physical_range': '[-327.68|327.67]', 'physical_unit': 'm/s',
// 'precision': 0.01, 'type': 'double'}
double Wheelspeedreport51e::rl(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 16, 'description': 'wheel speed front right',
// 'is_signed_var': True, 'len': 16, 'name': 'fr', 'offset': 0.0, 'order':
// 'intel', 'physical_range': '[-327.68|327.67]', 'physical_unit': 'm/s',
// 'precision': 0.01, 'type': 'double'}
double Wheelspeedreport51e::fr(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 0, 'description': 'wheel speed front left',
// 'is_signed_var': True, 'len': 16, 'name': 'fl', 'offset': 0.0, 'order':
// 'intel', 'physical_range': '[-327.68|327.67]', 'physical_unit': 'm/s',
// 'precision': 0.01, 'type': 'double'}
double Wheelspeedreport51e::fl(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
