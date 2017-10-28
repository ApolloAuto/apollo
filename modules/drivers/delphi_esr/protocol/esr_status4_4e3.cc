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

#include "modules/drivers/delphi_esr/protocol/esr_status4_4e3.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Esrstatus44e3::Esrstatus44e3() {}
const int32_t Esrstatus44e3::ID = 0x4E3;

void Esrstatus44e3::Parse(const std::uint8_t* bytes, int32_t length,
                          DelphiESR* delphi_esr) const {
  delphi_esr->mutable_esr_status4_4e3()->set_can_tx_truck_target_det(
      can_tx_truck_target_det(bytes, length));
  delphi_esr->mutable_esr_status4_4e3()->set_can_tx_lr_only_grating_lobe_det(
      can_tx_lr_only_grating_lobe_det(bytes, length));
  delphi_esr->mutable_esr_status4_4e3()->set_can_tx_sidelobe_blockage(
      can_tx_sidelobe_blockage(bytes, length));
  delphi_esr->mutable_esr_status4_4e3()->set_can_tx_partial_blockage(
      can_tx_partial_blockage(bytes, length));
  delphi_esr->mutable_esr_status4_4e3()->set_can_tx_path_id_acc_stat(
      can_tx_path_id_acc_stat(bytes, length));
  delphi_esr->mutable_esr_status4_4e3()->set_can_tx_mr_lr_mode(
      can_tx_mr_lr_mode(bytes, length));
  delphi_esr->mutable_esr_status4_4e3()->set_can_tx_auto_align_angle(
      can_tx_auto_align_angle(bytes, length));
  delphi_esr->mutable_esr_status4_4e3()->set_can_tx_rolling_count_3(
      can_tx_rolling_count_3(bytes, length));
  delphi_esr->mutable_esr_status4_4e3()->set_can_tx_path_id_fcw_stat(
      can_tx_path_id_fcw_stat(bytes, length));
  delphi_esr->mutable_esr_status4_4e3()->set_can_tx_path_id_fcw_move(
      can_tx_path_id_fcw_move(bytes, length));
  delphi_esr->mutable_esr_status4_4e3()->set_can_tx_path_id_cmbb_stat(
      can_tx_path_id_cmbb_stat(bytes, length));
  delphi_esr->mutable_esr_status4_4e3()->set_can_tx_path_id_cmbb_move(
      can_tx_path_id_cmbb_move(bytes, length));
  delphi_esr->mutable_esr_status4_4e3()->set_can_tx_path_id_acc(
      can_tx_path_id_acc(bytes, length));
}

// config detail: {'name': 'can_tx_truck_target_det', 'enum': {0:
// 'CAN_TX_TRUCK_TARGET_DET_NOT_DETECTED', 1:
// 'CAN_TX_TRUCK_TARGET_DET_DETECTED'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 7,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_status4_4e3::Can_tx_truck_target_detType
Esrstatus44e3::can_tx_truck_target_det(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  Esr_status4_4e3::Can_tx_truck_target_detType ret =
      static_cast<Esr_status4_4e3::Can_tx_truck_target_detType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_lr_only_grating_lobe_det', 'enum': {0:
// 'CAN_TX_LR_ONLY_GRATING_LOBE_DET_NOT_DETECTED', 1:
// 'CAN_TX_LR_ONLY_GRATING_LOBE_DET_DETECTED'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 6,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_status4_4e3::Can_tx_lr_only_grating_lobe_detType
Esrstatus44e3::can_tx_lr_only_grating_lobe_det(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 1);

  Esr_status4_4e3::Can_tx_lr_only_grating_lobe_detType ret =
      static_cast<Esr_status4_4e3::Can_tx_lr_only_grating_lobe_detType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_sidelobe_blockage', 'enum': {0:
// 'CAN_TX_SIDELOBE_BLOCKAGE_OFF', 1: 'CAN_TX_SIDELOBE_BLOCKAGE_ON'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 5, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Esr_status4_4e3::Can_tx_sidelobe_blockageType
Esrstatus44e3::can_tx_sidelobe_blockage(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  Esr_status4_4e3::Can_tx_sidelobe_blockageType ret =
      static_cast<Esr_status4_4e3::Can_tx_sidelobe_blockageType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_partial_blockage', 'enum': {0:
// 'CAN_TX_PARTIAL_BLOCKAGE_NOT_BLOCKED', 1: 'CAN_TX_PARTIAL_BLOCKAGE_BLOCKED'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 4, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Esr_status4_4e3::Can_tx_partial_blockageType
Esrstatus44e3::can_tx_partial_blockage(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 1);

  Esr_status4_4e3::Can_tx_partial_blockageType ret =
      static_cast<Esr_status4_4e3::Can_tx_partial_blockageType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_path_id_acc_stat', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus44e3::can_tx_path_id_acc_stat(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_mr_lr_mode', 'enum': {0:
// 'CAN_TX_MR_LR_MODE_RESERVED', 1:
// 'CAN_TX_MR_LR_MODE_OUTPUT_ONLY_MEDIUM_RANGE_TRACKS', 2:
// 'CAN_TX_MR_LR_MODE_OUTPUT_ONLY_LONG_RANGE_TRACKS', 3:
// 'CAN_TX_MR_LR_MODE_OUTPUT_ALL_MEDIUM_RANGE_AND_LONG'}, 'precision': 1.0,
// 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]',
// 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_status4_4e3::Can_tx_mr_lr_modeType Esrstatus44e3::can_tx_mr_lr_mode(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 2);

  Esr_status4_4e3::Can_tx_mr_lr_modeType ret =
      static_cast<Esr_status4_4e3::Can_tx_mr_lr_modeType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_auto_align_angle', 'offset': 0.0,
// 'precision': 0.0625, 'len': 8, 'is_signed_var': True, 'physical_range':
// '[-8|7.9375]', 'bit': 55, 'type': 'double', 'order': 'motorola',
// 'physical_unit': ''}
double Esrstatus44e3::can_tx_auto_align_angle(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.062500;
  return ret;
}

// config detail: {'name': 'can_tx_rolling_count_3', 'offset': 0.0, 'precision':
// 1.0, 'len': 2, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 1,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus44e3::can_tx_rolling_count_3(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_path_id_fcw_stat', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus44e3::can_tx_path_id_fcw_stat(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_path_id_fcw_move', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus44e3::can_tx_path_id_fcw_move(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_path_id_cmbb_stat', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus44e3::can_tx_path_id_cmbb_stat(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_path_id_cmbb_move', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus44e3::can_tx_path_id_cmbb_move(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_path_id_acc', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 15,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus44e3::can_tx_path_id_acc(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
