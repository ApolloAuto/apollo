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

#include "modules/drivers/delphi_esr/protocol/acm_inst_resp_7e4.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Acminstresp7e4::Acminstresp7e4() {}
const int32_t Acminstresp7e4::ID = 0x7E4;

void Acminstresp7e4::Parse(const std::uint8_t* bytes, int32_t length,
                           DelphiESR* delphi_esr) const {
  delphi_esr->mutable_acm_inst_resp_7e4()->set_data_7(data_7(bytes, length));
  delphi_esr->mutable_acm_inst_resp_7e4()->set_data_6(data_6(bytes, length));
  delphi_esr->mutable_acm_inst_resp_7e4()->set_data_5(data_5(bytes, length));
  delphi_esr->mutable_acm_inst_resp_7e4()->set_data_4(data_4(bytes, length));
  delphi_esr->mutable_acm_inst_resp_7e4()->set_data_3(data_3(bytes, length));
  delphi_esr->mutable_acm_inst_resp_7e4()->set_rtn_cmd_counter(
      rtn_cmd_counter(bytes, length));
  delphi_esr->mutable_acm_inst_resp_7e4()->set_command_return_code(
      command_return_code(bytes, length));
  delphi_esr->mutable_acm_inst_resp_7e4()->set_pid(pid(bytes, length));
}

// config detail: {'name': 'data_7', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 63, 'type': 'int',
// 'order': 'motorola', 'physical_unit': ''}
int Acminstresp7e4::data_7(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'data_6', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 55, 'type': 'int',
// 'order': 'motorola', 'physical_unit': ''}
int Acminstresp7e4::data_6(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'data_5', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 47, 'type': 'int',
// 'order': 'motorola', 'physical_unit': ''}
int Acminstresp7e4::data_5(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'data_4', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39, 'type': 'int',
// 'order': 'motorola', 'physical_unit': ''}
int Acminstresp7e4::data_4(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'data_3', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 31, 'type': 'int',
// 'order': 'motorola', 'physical_unit': ''}
int Acminstresp7e4::data_3(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rtn_cmd_counter', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 23,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Acminstresp7e4::rtn_cmd_counter(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'command_return_code', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 15,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Acminstresp7e4::command_return_code(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'pid', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7, 'type': 'int',
// 'order': 'motorola', 'physical_unit': ''}
int Acminstresp7e4::pid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
