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

#include "modules/drivers/delphi_esr/protocol/esr_status3_4e2.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Esrstatus34e2::Esrstatus34e2() {}
const int32_t Esrstatus34e2::ID = 0x4E2;

void Esrstatus34e2::Parse(const std::uint8_t* bytes, int32_t length,
                          DelphiESR* delphi_esr) const {
  delphi_esr->mutable_esr_status3_4e2()->set_can_tx_sw_version_pld(
      can_tx_sw_version_pld(bytes, length));
  delphi_esr->mutable_esr_status3_4e2()->set_can_tx_sw_version_host(
      can_tx_sw_version_host(bytes, length));
  delphi_esr->mutable_esr_status3_4e2()->set_can_tx_hw_version(
      can_tx_hw_version(bytes, length));
  delphi_esr->mutable_esr_status3_4e2()->set_can_tx_interface_version(
      can_tx_interface_version(bytes, length));
  delphi_esr->mutable_esr_status3_4e2()->set_can_tx_serial_num(
      can_tx_serial_num(bytes, length));
}

// config detail: {'name': 'can_tx_sw_version_pld', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 63,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus34e2::can_tx_sw_version_pld(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_sw_version_host', 'offset': 0.0, 'precision':
// 1.0, 'len': 24, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 15,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus34e2::can_tx_sw_version_host(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 3);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_hw_version', 'offset': 0.0, 'precision': 1.0,
// 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 3,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus34e2::can_tx_hw_version(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_interface_version', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus34e2::can_tx_interface_version(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_serial_num', 'offset': 0.0, 'precision': 1.0,
// 'len': 24, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus34e2::can_tx_serial_num(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 6);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
