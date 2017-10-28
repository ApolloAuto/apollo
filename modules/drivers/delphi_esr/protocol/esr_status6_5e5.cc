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

#include "modules/drivers/delphi_esr/protocol/esr_status6_5e5.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Esrstatus65e5::Esrstatus65e5() {}
const int32_t Esrstatus65e5::ID = 0x5E5;

void Esrstatus65e5::Parse(const std::uint8_t* bytes, int32_t length,
                          DelphiESR* delphi_esr) const {
  delphi_esr->mutable_esr_status6_5e5()->set_can_tx_sw_version_dsp_3rd_byte(
      can_tx_sw_version_dsp_3rd_byte(bytes, length));
  delphi_esr->mutable_esr_status6_5e5()->set_can_tx_vertical_align_updated(
      can_tx_vertical_align_updated(bytes, length));
  delphi_esr->mutable_esr_status6_5e5()->set_can_tx_vertical_misalignment(
      can_tx_vertical_misalignment(bytes, length));
  delphi_esr->mutable_esr_status6_5e5()->set_can_tx_serv_align_updates_done(
      can_tx_serv_align_updates_done(bytes, length));
  delphi_esr->mutable_esr_status6_5e5()->set_can_tx_found_target(
      can_tx_found_target(bytes, length));
  delphi_esr->mutable_esr_status6_5e5()->set_can_tx_factory_misalignment(
      can_tx_factory_misalignment(bytes, length));
  delphi_esr->mutable_esr_status6_5e5()->set_can_tx_factory_align_status_2(
      can_tx_factory_align_status_2(bytes, length));
  delphi_esr->mutable_esr_status6_5e5()->set_can_tx_factory_align_status_1(
      can_tx_factory_align_status_1(bytes, length));
  delphi_esr->mutable_esr_status6_5e5()->set_can_tx_recommend_unconverge(
      can_tx_recommend_unconverge(bytes, length));
  delphi_esr->mutable_esr_status6_5e5()->set_can_tx_wave_diff_a2d(
      can_tx_wave_diff_a2d(bytes, length));
  delphi_esr->mutable_esr_status6_5e5()->set_can_tx_system_power_mode(
      can_tx_system_power_mode(bytes, length));
  delphi_esr->mutable_esr_status6_5e5()->set_can_tx_supply_n5v_a2d(
      can_tx_supply_n5v_a2d(bytes, length));
  delphi_esr->mutable_esr_status6_5e5()->set_can_tx_supply_1p8v_a2d(
      can_tx_supply_1p8v_a2d(bytes, length));
}

// config detail: {'name': 'can_tx_sw_version_dsp_3rd_byte', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus65e5::can_tx_sw_version_dsp_3rd_byte(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_vertical_align_updated', 'enum': {0:
// 'CAN_TX_VERTICAL_ALIGN_UPDATED_NOT_UPDATED', 1:
// 'CAN_TX_VERTICAL_ALIGN_UPDATED_UPDATED'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 27,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_status6_5e5::Can_tx_vertical_align_updatedType
Esrstatus65e5::can_tx_vertical_align_updated(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(3, 1);

  Esr_status6_5e5::Can_tx_vertical_align_updatedType ret =
      static_cast<Esr_status6_5e5::Can_tx_vertical_align_updatedType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_vertical_misalignment', 'offset': 0.0,
// 'precision': 0.0625, 'len': 8, 'is_signed_var': True, 'physical_range':
// '[-6|6]', 'bit': 63, 'type': 'double', 'order': 'motorola', 'physical_unit':
// ''}
double Esrstatus65e5::can_tx_vertical_misalignment(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.062500;
  return ret;
}

// config detail: {'name': 'can_tx_serv_align_updates_done', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit':
// ''}
int Esrstatus65e5::can_tx_serv_align_updates_done(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_found_target', 'enum': {0:
// 'CAN_TX_FOUND_TARGET_NOT_FOUND', 1: 'CAN_TX_FOUND_TARGET_FOUND'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 39, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Esr_status6_5e5::Can_tx_found_targetType Esrstatus65e5::can_tx_found_target(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(7, 1);

  Esr_status6_5e5::Can_tx_found_targetType ret =
      static_cast<Esr_status6_5e5::Can_tx_found_targetType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_factory_misalignment', 'offset': 0.0,
// 'precision': 0.0625, 'len': 8, 'is_signed_var': True, 'physical_range':
// '[-5|5]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit':
// 'deg'}
double Esrstatus65e5::can_tx_factory_misalignment(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.062500;
  return ret;
}

// config detail: {'name': 'can_tx_factory_align_status_2', 'enum': {0:
// 'CAN_TX_FACTORY_ALIGN_STATUS_2_OFF', 1: 'CAN_TX_FACTORY_ALIGN_STATUS_2_BUSY',
// 2: 'CAN_TX_FACTORY_ALIGN_STATUS_2_SUCCESS', 3:
// 'CAN_TX_FACTORY_ALIGN_STATUS_2_FAIL_NO_TARGET', 4:
// 'CAN_TX_FACTORY_ALIGN_STATUS_2_FAIL_DEV_TOO_LARGE', 5:
// 'CAN_TX_FACTORY_ALIGN_STATUS_2_FAIL_VAR_TOO_LARGE'}, 'precision': 1.0, 'len':
// 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|5]', 'bit':
// 34, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_status6_5e5::Can_tx_factory_align_status_2Type
Esrstatus65e5::can_tx_factory_align_status_2(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 3);

  Esr_status6_5e5::Can_tx_factory_align_status_2Type ret =
      static_cast<Esr_status6_5e5::Can_tx_factory_align_status_2Type>(x);
  return ret;
}

// config detail: {'name': 'can_tx_factory_align_status_1', 'enum': {0:
// 'CAN_TX_FACTORY_ALIGN_STATUS_1_OFF', 1: 'CAN_TX_FACTORY_ALIGN_STATUS_1_BUSY',
// 2: 'CAN_TX_FACTORY_ALIGN_STATUS_1_SUCCESS', 3:
// 'CAN_TX_FACTORY_ALIGN_STATUS_1_FAIL_NO_TARGET', 4:
// 'CAN_TX_FACTORY_ALIGN_STATUS_1_FAIL_DEV_TOO_LARGE', 5:
// 'CAN_TX_FACTORY_ALIGN_STATUS_1_FAIL_VAR_TOO_LARGE'}, 'precision': 1.0, 'len':
// 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|5]', 'bit':
// 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_status6_5e5::Can_tx_factory_align_status_1Type
Esrstatus65e5::can_tx_factory_align_status_1(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 3);

  Esr_status6_5e5::Can_tx_factory_align_status_1Type ret =
      static_cast<Esr_status6_5e5::Can_tx_factory_align_status_1Type>(x);
  return ret;
}

// config detail: {'name': 'can_tx_recommend_unconverge', 'enum': {0:
// 'CAN_TX_RECOMMEND_UNCONVERGE_NOT_RECOMMEND', 1:
// 'CAN_TX_RECOMMEND_UNCONVERGE_RECOMMEND'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 38,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_status6_5e5::Can_tx_recommend_unconvergeType
Esrstatus65e5::can_tx_recommend_unconverge(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(6, 1);

  Esr_status6_5e5::Can_tx_recommend_unconvergeType ret =
      static_cast<Esr_status6_5e5::Can_tx_recommend_unconvergeType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_wave_diff_a2d', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 23,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus65e5::can_tx_wave_diff_a2d(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_system_power_mode', 'enum': {0:
// 'CAN_TX_SYSTEM_POWER_MODE_DSP_INIT', 1:
// 'CAN_TX_SYSTEM_POWER_MODE_RADIATE_OFF', 2:
// 'CAN_TX_SYSTEM_POWER_MODE_RADIATE_ON', 3:
// 'CAN_TX_SYSTEM_POWER_MODE_DSP_SHUTDOWN', 4:
// 'CAN_TX_SYSTEM_POWER_MODE_DSP_OFF', 5:
// 'CAN_TX_SYSTEM_POWER_MODE_HOST_SHUTDOWN', 6: 'CAN_TX_SYSTEM_POWER_MODE_TEST',
// 7: 'CAN_TX_SYSTEM_POWER_MODE_7INVALID'}, 'precision': 1.0, 'len': 3,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 26,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_status6_5e5::Can_tx_system_power_modeType
Esrstatus65e5::can_tx_system_power_mode(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 3);

  Esr_status6_5e5::Can_tx_system_power_modeType ret =
      static_cast<Esr_status6_5e5::Can_tx_system_power_modeType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_supply_n5v_a2d', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 15,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus65e5::can_tx_supply_n5v_a2d(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_supply_1p8v_a2d', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus65e5::can_tx_supply_1p8v_a2d(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
