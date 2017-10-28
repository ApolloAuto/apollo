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

#include "modules/drivers/delphi_esr/protocol/esr_valid2_5d1.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Esrvalid25d1::Esrvalid25d1() {}
const int32_t Esrvalid25d1::ID = 0x5D1;

void Esrvalid25d1::Parse(const std::uint8_t* bytes, int32_t length,
                         DelphiESR* delphi_esr) const {
  delphi_esr->mutable_esr_valid2_5d1()->set_can_tx_valid_mr_sn(
      can_tx_valid_mr_sn(bytes, length));
  delphi_esr->mutable_esr_valid2_5d1()->set_can_tx_valid_mr_range_rate(
      can_tx_valid_mr_range_rate(bytes, length));
  delphi_esr->mutable_esr_valid2_5d1()->set_can_tx_valid_mr_range(
      can_tx_valid_mr_range(bytes, length));
  delphi_esr->mutable_esr_valid2_5d1()->set_can_tx_valid_mr_power(
      can_tx_valid_mr_power(bytes, length));
  delphi_esr->mutable_esr_valid2_5d1()->set_can_tx_valid_mr_angle(
      can_tx_valid_mr_angle(bytes, length));
}

// config detail: {'name': 'can_tx_valid_mr_sn', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrvalid25d1::can_tx_valid_mr_sn(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_valid_mr_range_rate', 'offset': 0.0,
// 'precision': 0.0078125, 'len': 16, 'is_signed_var': True, 'physical_range':
// '[-128|127]', 'bit': 31, 'type': 'double', 'order': 'motorola',
// 'physical_unit': 'm/s'}
double Esrvalid25d1::can_tx_valid_mr_range_rate(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.007812;
  return ret;
}

// config detail: {'name': 'can_tx_valid_mr_range', 'offset': 0.0, 'precision':
// 0.0078125, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|200]',
// 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
double Esrvalid25d1::can_tx_valid_mr_range(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.007812;
  return ret;
}

// config detail: {'name': 'can_tx_valid_mr_power', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': True, 'physical_range': '[-10|40]', 'bit':
// 63, 'type': 'int', 'order': 'motorola', 'physical_unit': 'dB'}
int Esrvalid25d1::can_tx_valid_mr_power(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_valid_mr_angle', 'offset': 0.0, 'precision':
// 0.0625, 'len': 16, 'is_signed_var': True, 'physical_range': '[-64|63.9375]',
// 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Esrvalid25d1::can_tx_valid_mr_angle(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.062500;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
