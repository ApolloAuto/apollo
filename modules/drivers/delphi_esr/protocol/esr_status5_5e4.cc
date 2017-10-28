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

#include "modules/drivers/delphi_esr/protocol/esr_status5_5e4.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Esrstatus55e4::Esrstatus55e4() {}
const int32_t Esrstatus55e4::ID = 0x5E4;

void Esrstatus55e4::Parse(const std::uint8_t* bytes, int32_t length,
                          DelphiESR* delphi_esr) const {
  delphi_esr->mutable_esr_status5_5e4()->set_can_tx_supply_10v_a2d(
      can_tx_supply_10v_a2d(bytes, length));
  delphi_esr->mutable_esr_status5_5e4()->set_can_tx_temp2_a2d(
      can_tx_temp2_a2d(bytes, length));
  delphi_esr->mutable_esr_status5_5e4()->set_can_tx_temp1_a2d(
      can_tx_temp1_a2d(bytes, length));
  delphi_esr->mutable_esr_status5_5e4()->set_can_tx_swbatt_a2d(
      can_tx_swbatt_a2d(bytes, length));
  delphi_esr->mutable_esr_status5_5e4()->set_can_tx_supply_5vdx_a2d(
      can_tx_supply_5vdx_a2d(bytes, length));
  delphi_esr->mutable_esr_status5_5e4()->set_can_tx_supply_5va_a2d(
      can_tx_supply_5va_a2d(bytes, length));
  delphi_esr->mutable_esr_status5_5e4()->set_can_tx_supply_3p3v_a2d(
      can_tx_supply_3p3v_a2d(bytes, length));
  delphi_esr->mutable_esr_status5_5e4()->set_can_tx_ignp_a2d(
      can_tx_ignp_a2d(bytes, length));
}

// config detail: {'name': 'can_tx_supply_10v_a2d', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 63,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus55e4::can_tx_supply_10v_a2d(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_temp2_a2d', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 31,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus55e4::can_tx_temp2_a2d(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_temp1_a2d', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 23,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus55e4::can_tx_temp1_a2d(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_swbatt_a2d', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus55e4::can_tx_swbatt_a2d(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_supply_5vdx_a2d', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 47,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus55e4::can_tx_supply_5vdx_a2d(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_supply_5va_a2d', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus55e4::can_tx_supply_5va_a2d(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_supply_3p3v_a2d', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 55,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus55e4::can_tx_supply_3p3v_a2d(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_ignp_a2d', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 15,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus55e4::can_tx_ignp_a2d(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
