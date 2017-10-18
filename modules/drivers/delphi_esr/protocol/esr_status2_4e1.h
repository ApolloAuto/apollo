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

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_STATUS2_4E1_H_
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_STATUS2_4E1_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::DelphiESR;

class Esrstatus24e1 : public apollo::drivers::canbus::ProtocolData<DelphiESR> {
 public:
  static const int32_t ID;
  Esrstatus24e1();
  void Parse(const std::uint8_t* bytes, int32_t length,
             DelphiESR* delphi_esr) const override;

 private:
  // config detail: {'name': 'CAN_TX_YAW_RATE_BIAS', 'offset': 0.0, 'precision':
  // 0.125, 'len': 8, 'is_signed_var': True, 'physical_range': '[-16|15.875]',
  // 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  double can_tx_yaw_rate_bias(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'CAN_TX_VEH_SPD_COMP_FACTOR', 'offset': 1.0,
  // 'precision': 0.001953125, 'len': 6, 'is_signed_var': True,
  // 'physical_range': '[0.9375|1.060546875]', 'bit': 39, 'type': 'double',
  // 'order': 'motorola', 'physical_unit': ''}
  double can_tx_veh_spd_comp_factor(const std::uint8_t* bytes,
                                    const int32_t length) const;

  // config detail: {'name': 'CAN_TX_SW_VERSION_DSP', 'offset': 0.0,
  // 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_sw_version_dsp(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'CAN_TX_TEMPERATURE', 'offset': 0.0, 'precision':
  // 1.0, 'len': 8, 'is_signed_var': True, 'physical_range': '[-128|127]',
  // 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': 'degC'}
  int can_tx_temperature(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_RAW_DATA_MODE', 'enum': {0:
  // 'CAN_TX_RAW_DATA_MODE_FILTERED', 1: 'CAN_TX_RAW_DATA_MODE_RAW'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 11, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Esr_status2_4e1::Can_tx_raw_data_modeType can_tx_raw_data_mode(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_RANGE_PERF_ERROR', 'enum': {0:
  // 'CAN_TX_RANGE_PERF_ERROR_NOT_BLOCKED', 1:
  // 'CAN_TX_RANGE_PERF_ERROR_BLOCKED'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_status2_4e1::Can_tx_range_perf_errorType can_tx_range_perf_error(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_OVERHEAT_ERROR', 'enum': {0:
  // 'CAN_TX_OVERHEAT_ERROR_NOT_OVERTEMP', 1: 'CAN_TX_OVERHEAT_ERROR_OVERTEMP'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 15, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Esr_status2_4e1::Can_tx_overheat_errorType can_tx_overheat_error(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_MAXIMUM_TRACKS_ACK', 'offset': 1.0,
  // 'precision': 1.0, 'len': 6, 'is_signed_var': False, 'physical_range':
  // '[1|64]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_tx_maximum_tracks_ack(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'name': 'CAN_TX_INTERNAL_ERROR', 'enum': {0:
  // 'CAN_TX_INTERNAL_ERROR_NOT_FAILED', 1: 'CAN_TX_INTERNAL_ERROR_FAILED'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 13, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Esr_status2_4e1::Can_tx_internal_errorType can_tx_internal_error(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_GROUPING_MODE', 'enum': {0:
  // 'CAN_TX_GROUPING_MODE_NO_GROUPING', 1:
  // 'CAN_TX_GROUPING_MODE_GROUP_MOVING_ONLY', 2:
  // 'CAN_TX_GROUPING_MODE_GROUP_STATIONARY_ONLY', 3:
  // 'CAN_TX_GROUPING_MODE_GROUP_MOVING_STATIONARY'}, 'precision': 1.0, 'len':
  // 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 33, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_status2_4e1::Can_tx_grouping_modeType can_tx_grouping_mode(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_XCVR_OPERATIONAL', 'enum': {0:
  // 'CAN_TX_XCVR_OPERATIONAL_OFF', 1: 'CAN_TX_XCVR_OPERATIONAL_ON'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 12, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Esr_status2_4e1::Can_tx_xcvr_operationalType can_tx_xcvr_operational(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_STEERING_ANGLE_ACK', 'offset': 0.0,
  // 'precision': 1.0, 'len': 11, 'is_signed_var': False, 'physical_range':
  // '[0|2047]', 'bit': 10, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // 'deg'}
  int can_tx_steering_angle_ack(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'name': 'CAN_TX_ROLLING_COUNT_2', 'offset': 0.0,
  // 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 1, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int can_tx_rolling_count_2(const std::uint8_t* bytes,
                             const int32_t length) const;
};

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_ESR_STATUS2_4E1_H_
