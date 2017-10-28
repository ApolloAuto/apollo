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

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_STATUS9_5E8_H_
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_STATUS9_5E8_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::DelphiESR;

class Esrstatus95e8 : public apollo::drivers::canbus::ProtocolData<DelphiESR> {
 public:
  static const int32_t ID;
  Esrstatus95e8();
  void Parse(const std::uint8_t* bytes, int32_t length,
             DelphiESR* delphi_esr) const override;

 private:
  // config detail: {'name': 'CAN_TX_PATH_ID_ACC_3', 'offset': 0.0, 'precision':
  // 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|64]', 'bit':
  // 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int can_tx_path_id_acc_3(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'name': 'CAN_TX_PATH_ID_ACC_2', 'offset': 0.0, 'precision':
  // 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|64]', 'bit':
  // 55, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int can_tx_path_id_acc_2(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'name': 'CAN_TX_FILTERED_XOHP_ACC_CIPV', 'offset': 0.0,
  // 'precision': 0.03125, 'len': 9, 'is_signed_var': True, 'physical_range':
  // '[-8|7.96875]', 'bit': 32, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm'}
  double can_tx_filtered_xohp_acc_cipv(const std::uint8_t* bytes,
                                       const int32_t length) const;

  // config detail: {'name': 'CAN_TX_WATER_SPRAY_TARGET_ID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 7, 'is_signed_var': False, 'physical_range':
  // '[0|64]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_water_spray_target_id(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'name': 'CAN_TX_SERIAL_NUM_3RD_BYTE', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_serial_num_3rd_byte(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'name': 'CAN_TX_SIDESLIP_ANGLE', 'offset': 0.0,
  // 'precision': 0.125, 'len': 10, 'is_signed_var': True, 'physical_range':
  // '[-64|63.875]', 'bit': 9, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'deg'}
  double can_tx_sideslip_angle(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'name': 'CAN_TX_AVG_PWR_CWBLKG', 'offset': 0.0,
  // 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int can_tx_avg_pwr_cwblkg(const std::uint8_t* bytes,
                            const int32_t length) const;
};

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_ESR_STATUS9_5E8_H_
