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

#include "modules/drivers/delphi_esr/protocol/vehicle6_5f5.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Vehicle65f5::Vehicle65f5() {}
const int32_t Vehicle65f5::ID = 0x5F5;

void Vehicle65f5::Parse(const std::uint8_t* bytes, int32_t length,
                        DelphiESR* delphi_esr) const {
  delphi_esr->mutable_vehicle6_5f5()->set_can_rx_inner_funnel_offset_right(
      can_rx_inner_funnel_offset_right(bytes, length));
  delphi_esr->mutable_vehicle6_5f5()->set_can_rx_inner_funnel_offset_left(
      can_rx_inner_funnel_offset_left(bytes, length));
  delphi_esr->mutable_vehicle6_5f5()->set_can_volvo_fa_range_max_short(
      can_volvo_fa_range_max_short(bytes, length));
  delphi_esr->mutable_vehicle6_5f5()->set_can_volvo_fa_min_vspeed_short(
      can_volvo_fa_min_vspeed_short(bytes, length));
  delphi_esr->mutable_vehicle6_5f5()->set_can_volvo_fa_aalign_estimate(
      can_volvo_fa_aalign_estimate(bytes, length));
}

// config detail: {'name': 'can_rx_inner_funnel_offset_right', 'offset': 0.0,
// 'precision': 0.1, 'len': 8, 'is_signed_var': True, 'physical_range':
// '[-2|10]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit':
// 'm'}
double Vehicle65f5::can_rx_inner_funnel_offset_right(const std::uint8_t* bytes,
                                                     int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'name': 'can_rx_inner_funnel_offset_left', 'offset': 0.0,
// 'precision': 0.1, 'len': 8, 'is_signed_var': True, 'physical_range':
// '[-2|10]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit':
// 'm'}
double Vehicle65f5::can_rx_inner_funnel_offset_left(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'name': 'can_volvo_fa_range_max_short', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit':
// 'm'}
int Vehicle65f5::can_volvo_fa_range_max_short(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_volvo_fa_min_vspeed_short', 'offset': 0.0,
// 'precision': 0.125, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|20]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit':
// 'm/s'}
double Vehicle65f5::can_volvo_fa_min_vspeed_short(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.125000;
  return ret;
}

// config detail: {'name': 'can_volvo_fa_aalign_estimate', 'offset': 0.0,
// 'precision': 0.0625, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|10]', 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit':
// 'deg'}
double Vehicle65f5::can_volvo_fa_aalign_estimate(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.062500;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
