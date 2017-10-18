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

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_VEHICLE4_5F3_H_
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_VEHICLE4_5F3_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::DelphiESR;

class Vehicle45f3 : public apollo::drivers::canbus::ProtocolData<DelphiESR> {
 public:
  static const int32_t ID;
  Vehicle45f3();
  void Parse(const std::uint8_t* bytes, int32_t length,
             DelphiESR* delphi_esr) const override;

 private:
  // config detail: {'name': 'CAN_RX_FAC_TGT_RANGE_R2M', 'offset': 0.0,
  // 'precision': 0.0625, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[2|10]', 'bit': 55, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm'}
  double can_rx_fac_tgt_range_r2m(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'name': 'CAN_RX_FAC_TGT_RANGE_M2T', 'offset': 0.0,
  // 'precision': 0.0625, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[1|10]', 'bit': 63, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm'}
  double can_rx_fac_tgt_range_m2t(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'name': 'CAN_RX_FAC_TGT_RANGE_1', 'offset': 0.0,
  // 'precision': 0.0625, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[2|10]', 'bit': 47, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm'}
  double can_rx_fac_tgt_range_1(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'name': 'CAN_RX_FAC_TGT_MTG_SPACE_VER', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-100|100]', 'bit': 39, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'cm'}
  int can_rx_fac_tgt_mtg_space_ver(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'name': 'CAN_RX_FAC_TGT_MTG_SPACE_HOR', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-100|100]', 'bit': 31, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'cm'}
  int can_rx_fac_tgt_mtg_space_hor(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'name': 'CAN_RX_FAC_TGT_MTG_OFFSET', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-100|100]', 'bit': 23, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'cm'}
  int can_rx_fac_tgt_mtg_offset(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'name': 'CAN_RX_FAC_ALIGN_SAMP_REQ', 'offset': 0.0,
  // 'precision': 1.0, 'len': 7, 'is_signed_var': False, 'physical_range':
  // '[0|100]', 'bit': 14, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_rx_fac_align_samp_req(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'name': 'CAN_RX_FAC_ALIGN_MAX_NT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 7, 'is_signed_var': False, 'physical_range':
  // '[0|100]', 'bit': 6, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_rx_fac_align_max_nt(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'CAN_RX_FAC_ALIGN_CMD_2', 'enum': {0:
  // 'CAN_RX_FAC_ALIGN_CMD_2_OFF', 1: 'CAN_RX_FAC_ALIGN_CMD_2_ON'}, 'precision':
  // 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range':
  // '[0|0]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit':
  // ''}
  Vehicle4_5f3::Can_rx_fac_align_cmd_2Type can_rx_fac_align_cmd_2(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_FAC_ALIGN_CMD_1', 'enum': {0:
  // 'CAN_RX_FAC_ALIGN_CMD_1_OFF', 1: 'CAN_RX_FAC_ALIGN_CMD_1_ON'}, 'precision':
  // 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range':
  // '[0|0]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit':
  // ''}
  Vehicle4_5f3::Can_rx_fac_align_cmd_1Type can_rx_fac_align_cmd_1(
      const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_VEHICLE4_5F3_H_
