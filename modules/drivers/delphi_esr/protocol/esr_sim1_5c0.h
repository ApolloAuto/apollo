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

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_SIM1_5C0_H_
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_SIM1_5C0_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::DelphiESR;

class Esrsim15c0 : public apollo::drivers::canbus::ProtocolData<DelphiESR> {
 public:
  static const int32_t ID;
  Esrsim15c0();
  void Parse(const std::uint8_t* bytes, int32_t length,
             DelphiESR* delphi_esr) const override;

 private:
  // config detail: {'name': 'CAN_RX_SIM_TRACK_ID', 'enum': {0:
  // 'CAN_RX_SIM_TRACK_ID_NO_TARGET', 1: 'CAN_RX_SIM_TRACK_ID_TARGET_1', 2:
  // 'CAN_RX_SIM_TRACK_ID_TARGET_2'}, 'precision': 1.0, 'len': 2,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 6,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_sim1_5c0::Can_rx_sim_track_idType can_rx_sim_track_id(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_SIM_STATUS', 'enum': {0:
  // 'CAN_RX_SIM_STATUS_INVALID', 1: 'CAN_RX_SIM_STATUS_NEW', 2:
  // 'CAN_RX_SIM_STATUS_UPDATED', 3: 'CAN_RX_SIM_STATUS_COASTED'}, 'precision':
  // 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range':
  // '[0|0]', 'bit': 4, 'type': 'enum', 'order': 'motorola', 'physical_unit':
  // ''}
  Esr_sim1_5c0::Can_rx_sim_statusType can_rx_sim_status(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_SIM_RANGE_RATE', 'offset': 0.0,
  // 'precision': 0.25, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-32|31.75]', 'bit': 55, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm/s'}
  double can_rx_sim_range_rate(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'name': 'CAN_RX_SIM_RANGE_ACCEL', 'offset': 0.0,
  // 'precision': 0.25, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-32|31.75]', 'bit': 47, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm/s/s'}
  double can_rx_sim_range_accel(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'name': 'CAN_RX_SIM_RANGE', 'offset': 0.0, 'precision':
  // 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
  // 39, 'type': 'int', 'order': 'motorola', 'physical_unit': 'm'}
  int can_rx_sim_range(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_SIM_LAT_RATE', 'offset': 0.0, 'precision':
  // 0.25, 'len': 8, 'is_signed_var': True, 'physical_range': '[-32|31.75]',
  // 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
  double can_rx_sim_lat_rate(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'name': 'CAN_RX_SIM_LAT_POS', 'offset': 0.0, 'precision':
  // 0.25, 'len': 8, 'is_signed_var': True, 'physical_range': '[-32|31.75]',
  // 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
  double can_rx_sim_lat_pos(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'CAN_RX_SIM_FUNCTION', 'enum': {0:
  // 'CAN_RX_SIM_FUNCTION_ACC', 1: 'CAN_RX_SIM_FUNCTION_RI', 2:
  // 'CAN_RX_SIM_FUNCTION_FCW_MOVE', 3: 'CAN_RX_SIM_FUNCTION_FCW_STAT', 4:
  // 'CAN_RX_SIM_FUNCTION_CMBB_MOVE', 5: 'CAN_RX_SIM_FUNCTION_CMBB_STAT', 6:
  // 'CAN_RX_SIM_FUNCTION_ALL_MOVING_ACC_FCW_CMBB', 7:
  // 'CAN_RX_SIM_FUNCTION_ALL_STAT_RI_FCW_CMBB'}, 'precision': 1.0, 'len': 3,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 2,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_sim1_5c0::Can_rx_sim_functionType can_rx_sim_function(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_SIM_ANGLE', 'offset': 0.0, 'precision':
  // 0.5, 'len': 8, 'is_signed_var': True, 'physical_range': '[-64|63.5]',
  // 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
  double can_rx_sim_angle(const std::uint8_t* bytes,
                          const int32_t length) const;
};

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_ESR_SIM1_5C0_H_
