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

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_STATUS6_5E5_H_
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_STATUS6_5E5_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::DelphiESR;

class Esrstatus65e5 : public apollo::drivers::canbus::ProtocolData<DelphiESR> {
 public:
  static const int32_t ID;
  Esrstatus65e5();
  void Parse(const std::uint8_t* bytes, int32_t length,
             DelphiESR* delphi_esr) const override;

 private:
  // config detail: {'name': 'CAN_TX_SW_VERSION_DSP_3RD_BYTE', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_sw_version_dsp_3rd_byte(const std::uint8_t* bytes,
                                     const int32_t length) const;

  // config detail: {'name': 'CAN_TX_VERTICAL_ALIGN_UPDATED', 'enum': {0:
  // 'CAN_TX_VERTICAL_ALIGN_UPDATED_NOT_UPDATED', 1:
  // 'CAN_TX_VERTICAL_ALIGN_UPDATED_UPDATED'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 27, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_status6_5e5::Can_tx_vertical_align_updatedType
  can_tx_vertical_align_updated(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'name': 'CAN_TX_VERTICAL_MISALIGNMENT', 'offset': 0.0,
  // 'precision': 0.0625, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-6|6]', 'bit': 63, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': ''}
  double can_tx_vertical_misalignment(const std::uint8_t* bytes,
                                      const int32_t length) const;

  // config detail: {'name': 'CAN_TX_SERV_ALIGN_UPDATES_DONE', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_serv_align_updates_done(const std::uint8_t* bytes,
                                     const int32_t length) const;

  // config detail: {'name': 'CAN_TX_FOUND_TARGET', 'enum': {0:
  // 'CAN_TX_FOUND_TARGET_NOT_FOUND', 1: 'CAN_TX_FOUND_TARGET_FOUND'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 39, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Esr_status6_5e5::Can_tx_found_targetType can_tx_found_target(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_FACTORY_MISALIGNMENT', 'offset': 0.0,
  // 'precision': 0.0625, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-5|5]', 'bit': 47, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'deg'}
  double can_tx_factory_misalignment(const std::uint8_t* bytes,
                                     const int32_t length) const;

  // config detail: {'name': 'CAN_TX_FACTORY_ALIGN_STATUS_2', 'enum': {0:
  // 'CAN_TX_FACTORY_ALIGN_STATUS_2_OFF', 1:
  // 'CAN_TX_FACTORY_ALIGN_STATUS_2_BUSY', 2:
  // 'CAN_TX_FACTORY_ALIGN_STATUS_2_SUCCESS', 3:
  // 'CAN_TX_FACTORY_ALIGN_STATUS_2_FAIL_NO_TARGET', 4:
  // 'CAN_TX_FACTORY_ALIGN_STATUS_2_FAIL_DEV_TOO_LARGE', 5:
  // 'CAN_TX_FACTORY_ALIGN_STATUS_2_FAIL_VAR_TOO_LARGE'}, 'precision': 1.0,
  // 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|5]',
  // 'bit': 34, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_status6_5e5::Can_tx_factory_align_status_2Type
  can_tx_factory_align_status_2(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'name': 'CAN_TX_FACTORY_ALIGN_STATUS_1', 'enum': {0:
  // 'CAN_TX_FACTORY_ALIGN_STATUS_1_OFF', 1:
  // 'CAN_TX_FACTORY_ALIGN_STATUS_1_BUSY', 2:
  // 'CAN_TX_FACTORY_ALIGN_STATUS_1_SUCCESS', 3:
  // 'CAN_TX_FACTORY_ALIGN_STATUS_1_FAIL_NO_TARGET', 4:
  // 'CAN_TX_FACTORY_ALIGN_STATUS_1_FAIL_DEV_TOO_LARGE', 5:
  // 'CAN_TX_FACTORY_ALIGN_STATUS_1_FAIL_VAR_TOO_LARGE'}, 'precision': 1.0,
  // 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|5]',
  // 'bit': 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_status6_5e5::Can_tx_factory_align_status_1Type
  can_tx_factory_align_status_1(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'name': 'CAN_TX_RECOMMEND_UNCONVERGE', 'enum': {0:
  // 'CAN_TX_RECOMMEND_UNCONVERGE_NOT_RECOMMEND', 1:
  // 'CAN_TX_RECOMMEND_UNCONVERGE_RECOMMEND'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 38, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_status6_5e5::Can_tx_recommend_unconvergeType can_tx_recommend_unconverge(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_WAVE_DIFF_A2D', 'offset': 0.0, 'precision':
  // 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
  // 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int can_tx_wave_diff_a2d(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'name': 'CAN_TX_SYSTEM_POWER_MODE', 'enum': {0:
  // 'CAN_TX_SYSTEM_POWER_MODE_DSP_INIT', 1:
  // 'CAN_TX_SYSTEM_POWER_MODE_RADIATE_OFF', 2:
  // 'CAN_TX_SYSTEM_POWER_MODE_RADIATE_ON', 3:
  // 'CAN_TX_SYSTEM_POWER_MODE_DSP_SHUTDOWN', 4:
  // 'CAN_TX_SYSTEM_POWER_MODE_DSP_OFF', 5:
  // 'CAN_TX_SYSTEM_POWER_MODE_HOST_SHUTDOWN', 6:
  // 'CAN_TX_SYSTEM_POWER_MODE_TEST', 7: 'CAN_TX_SYSTEM_POWER_MODE_7INVALID'},
  // 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 26, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Esr_status6_5e5::Can_tx_system_power_modeType can_tx_system_power_mode(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_SUPPLY_N5V_A2D', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_supply_n5v_a2d(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'CAN_TX_SUPPLY_1P8V_A2D', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int can_tx_supply_1p8v_a2d(const std::uint8_t* bytes,
                             const int32_t length) const;
};

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_ESR_STATUS6_5E5_H_
