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

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ACM_INST_REQ_7E0_H_
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ACM_INST_REQ_7E0_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::DelphiESR;

class Acminstreq7e0 : public apollo::drivers::canbus::ProtocolData<DelphiESR> {
 public:
  static const int32_t ID;
  Acminstreq7e0();
  void Parse(const std::uint8_t* bytes, int32_t length,
             DelphiESR* delphi_esr) const override;

 private:
  // config detail: {'name': 'command_ctr', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 15,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int command_ctr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'command_code', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int command_code(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'cc_word_2', 'offset': 0.0, 'precision': 1.0,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 55,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int cc_word_2(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'cc_word_1', 'offset': 0.0, 'precision': 1.0,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int cc_word_1(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'cc_byte_2', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 31,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int cc_byte_2(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'cc_byte_1', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 23,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int cc_byte_1(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_ACM_INST_REQ_7E0_H_
