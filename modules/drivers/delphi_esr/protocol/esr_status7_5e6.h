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

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_STATUS7_5E6_H_
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_STATUS7_5E6_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::DelphiESR;

class Esrstatus75e6 : public apollo::drivers::canbus::ProtocolData<DelphiESR> {
 public:
  static const int32_t ID;
  Esrstatus75e6();
  void Parse(const std::uint8_t* bytes, int32_t length,
             DelphiESR* delphi_esr) const override;

 private:
  // config detail: {'name': 'CAN_TX_ACTIVE_FAULT_7', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_active_fault_7(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'CAN_TX_ACTIVE_FAULT_6', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_active_fault_6(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'CAN_TX_ACTIVE_FAULT_5', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_active_fault_5(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'CAN_TX_ACTIVE_FAULT_4', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_active_fault_4(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'CAN_TX_ACTIVE_FAULT_3', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_active_fault_3(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'CAN_TX_ACTIVE_FAULT_2', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_active_fault_2(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'CAN_TX_ACTIVE_FAULT_0', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int can_tx_active_fault_0(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'CAN_TX_ACTIVE_FAULT_1', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_active_fault_1(const std::uint8_t* bytes,
                            const int32_t length) const;
};

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_ESR_STATUS7_5E6_H_
