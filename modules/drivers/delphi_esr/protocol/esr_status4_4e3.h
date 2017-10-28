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

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_STATUS4_4E3_H_
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_STATUS4_4E3_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::DelphiESR;

class Esrstatus44e3 : public apollo::drivers::canbus::ProtocolData<DelphiESR> {
 public:
  static const int32_t ID;
  Esrstatus44e3();
  void Parse(const std::uint8_t* bytes, int32_t length,
             DelphiESR* delphi_esr) const override;

 private:
  // config detail: {'name': 'CAN_TX_TRUCK_TARGET_DET', 'enum': {0:
  // 'CAN_TX_TRUCK_TARGET_DET_NOT_DETECTED', 1:
  // 'CAN_TX_TRUCK_TARGET_DET_DETECTED'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 7,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_status4_4e3::Can_tx_truck_target_detType can_tx_truck_target_det(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_LR_ONLY_GRATING_LOBE_DET', 'enum': {0:
  // 'CAN_TX_LR_ONLY_GRATING_LOBE_DET_NOT_DETECTED', 1:
  // 'CAN_TX_LR_ONLY_GRATING_LOBE_DET_DETECTED'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 6,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_status4_4e3::Can_tx_lr_only_grating_lobe_detType
  can_tx_lr_only_grating_lobe_det(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'name': 'CAN_TX_SIDELOBE_BLOCKAGE', 'enum': {0:
  // 'CAN_TX_SIDELOBE_BLOCKAGE_OFF', 1: 'CAN_TX_SIDELOBE_BLOCKAGE_ON'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 5, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Esr_status4_4e3::Can_tx_sidelobe_blockageType can_tx_sidelobe_blockage(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_PARTIAL_BLOCKAGE', 'enum': {0:
  // 'CAN_TX_PARTIAL_BLOCKAGE_NOT_BLOCKED', 1:
  // 'CAN_TX_PARTIAL_BLOCKAGE_BLOCKED'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 4,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_status4_4e3::Can_tx_partial_blockageType can_tx_partial_blockage(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_PATH_ID_ACC_STAT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_path_id_acc_stat(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'CAN_TX_MR_LR_MODE', 'enum': {0:
  // 'CAN_TX_MR_LR_MODE_RESERVED', 1:
  // 'CAN_TX_MR_LR_MODE_OUTPUT_ONLY_MEDIUM_RANGE_TRACKS', 2:
  // 'CAN_TX_MR_LR_MODE_OUTPUT_ONLY_LONG_RANGE_TRACKS', 3:
  // 'CAN_TX_MR_LR_MODE_OUTPUT_ALL_MEDIUM_RANGE_AND_LONG'}, 'precision': 1.0,
  // 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]',
  // 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_status4_4e3::Can_tx_mr_lr_modeType can_tx_mr_lr_mode(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_AUTO_ALIGN_ANGLE', 'offset': 0.0,
  // 'precision': 0.0625, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-8|7.9375]', 'bit': 55, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': ''}
  double can_tx_auto_align_angle(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'name': 'CAN_TX_ROLLING_COUNT_3', 'offset': 0.0,
  // 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 1, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int can_tx_rolling_count_3(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'name': 'CAN_TX_PATH_ID_FCW_STAT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_path_id_fcw_stat(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'CAN_TX_PATH_ID_FCW_MOVE', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_path_id_fcw_move(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'CAN_TX_PATH_ID_CMBB_STAT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_path_id_cmbb_stat(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'name': 'CAN_TX_PATH_ID_CMBB_MOVE', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_path_id_cmbb_move(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'name': 'CAN_TX_PATH_ID_ACC', 'offset': 0.0, 'precision':
  // 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
  // 15, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int can_tx_path_id_acc(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_ESR_STATUS4_4E3_H_
