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

#include "modules/drivers/delphi_esr/protocol/acm_inst_req_7e0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Acminstreq7e0::Acminstreq7e0() {}
const int32_t Acminstreq7e0::ID = 0x7E0;

void Acminstreq7e0::Parse(const std::uint8_t* bytes, int32_t length,
                          DelphiESR* delphi_esr) const {
  delphi_esr->mutable_acm_inst_req_7e0()->set_command_ctr(
      command_ctr(bytes, length));
  delphi_esr->mutable_acm_inst_req_7e0()->set_command_code(
      command_code(bytes, length));
  delphi_esr->mutable_acm_inst_req_7e0()->set_cc_word_2(
      cc_word_2(bytes, length));
  delphi_esr->mutable_acm_inst_req_7e0()->set_cc_word_1(
      cc_word_1(bytes, length));
  delphi_esr->mutable_acm_inst_req_7e0()->set_cc_byte_2(
      cc_byte_2(bytes, length));
  delphi_esr->mutable_acm_inst_req_7e0()->set_cc_byte_1(
      cc_byte_1(bytes, length));
}

// config detail: {'name': 'command_ctr', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 15,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Acminstreq7e0::command_ctr(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'command_code', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Acminstreq7e0::command_code(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'cc_word_2', 'offset': 0.0, 'precision': 1.0, 'len':
// 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 55, 'type':
// 'int', 'order': 'motorola', 'physical_unit': ''}
int Acminstreq7e0::cc_word_2(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'cc_word_1', 'offset': 0.0, 'precision': 1.0, 'len':
// 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39, 'type':
// 'int', 'order': 'motorola', 'physical_unit': ''}
int Acminstreq7e0::cc_word_1(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'cc_byte_2', 'offset': 0.0, 'precision': 1.0, 'len':
// 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 31, 'type':
// 'int', 'order': 'motorola', 'physical_unit': ''}
int Acminstreq7e0::cc_byte_2(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'cc_byte_1', 'offset': 0.0, 'precision': 1.0, 'len':
// 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 23, 'type':
// 'int', 'order': 'motorola', 'physical_unit': ''}
int Acminstreq7e0::cc_byte_1(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
