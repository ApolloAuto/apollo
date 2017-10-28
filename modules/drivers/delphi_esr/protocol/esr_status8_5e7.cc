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

#include "modules/drivers/delphi_esr/protocol/esr_status8_5e7.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Esrstatus85e7::Esrstatus85e7() {}
const int32_t Esrstatus85e7::ID = 0x5E7;

void Esrstatus85e7::Parse(const std::uint8_t* bytes, int32_t length,
                          DelphiESR* delphi_esr) const {
  delphi_esr->mutable_esr_status8_5e7()->set_can_tx_history_fault_7(
      can_tx_history_fault_7(bytes, length));
  delphi_esr->mutable_esr_status8_5e7()->set_can_tx_history_fault_6(
      can_tx_history_fault_6(bytes, length));
  delphi_esr->mutable_esr_status8_5e7()->set_can_tx_history_fault_5(
      can_tx_history_fault_5(bytes, length));
  delphi_esr->mutable_esr_status8_5e7()->set_can_tx_history_fault_4(
      can_tx_history_fault_4(bytes, length));
  delphi_esr->mutable_esr_status8_5e7()->set_can_tx_history_fault_3(
      can_tx_history_fault_3(bytes, length));
  delphi_esr->mutable_esr_status8_5e7()->set_can_tx_history_fault_2(
      can_tx_history_fault_2(bytes, length));
  delphi_esr->mutable_esr_status8_5e7()->set_can_tx_history_fault_1(
      can_tx_history_fault_1(bytes, length));
  delphi_esr->mutable_esr_status8_5e7()->set_can_tx_history_fault_0(
      can_tx_history_fault_0(bytes, length));
}

// config detail: {'name': 'can_tx_history_fault_7', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 63,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus85e7::can_tx_history_fault_7(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_history_fault_6', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 55,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus85e7::can_tx_history_fault_6(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_history_fault_5', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 47,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus85e7::can_tx_history_fault_5(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_history_fault_4', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus85e7::can_tx_history_fault_4(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_history_fault_3', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 31,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus85e7::can_tx_history_fault_3(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_history_fault_2', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 23,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus85e7::can_tx_history_fault_2(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_history_fault_1', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 15,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus85e7::can_tx_history_fault_1(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_history_fault_0', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus85e7::can_tx_history_fault_0(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
