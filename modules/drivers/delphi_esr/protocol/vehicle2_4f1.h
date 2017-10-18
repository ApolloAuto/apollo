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

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_VEHICLE2_4F1_H_
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_VEHICLE2_4F1_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::DelphiESR;

class Vehicle24f1 : public apollo::drivers::canbus::ProtocolData<DelphiESR> {
 public:
  static const int32_t ID;
  Vehicle24f1();
  void Parse(const std::uint8_t* bytes, int32_t length,
             DelphiESR* delphi_esr) const override;

 private:
  // config detail: {'name': 'CAN_RX_VOLVO_SHORT_TRACK_ROC', 'offset': 0.0,
  // 'precision': 500.0, 'len': 4, 'is_signed_var': True, 'physical_range':
  // '[-4000|3500]', 'bit': 31, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm'}
  double can_rx_volvo_short_track_roc(const std::uint8_t* bytes,
                                      const int32_t length) const;

  // config detail: {'name': 'CAN_RX_MR_ONLY_TRANSMIT', 'enum': {0:
  // 'CAN_RX_MR_ONLY_TRANSMIT_OFF', 1: 'CAN_RX_MR_ONLY_TRANSMIT_ON'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 25, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Vehicle2_4f1::Can_rx_mr_only_transmitType can_rx_mr_only_transmit(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_LR_ONLY_TRANSMIT', 'enum': {0:
  // 'CAN_RX_LR_ONLY_TRANSMIT_OFF', 1: 'CAN_RX_LR_ONLY_TRANSMIT_ON'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 24, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Vehicle2_4f1::Can_rx_lr_only_transmitType can_rx_lr_only_transmit(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_HIGH_YAW_ANGLE', 'offset': 0.0,
  // 'precision': 1.0, 'len': 6, 'is_signed_var': True, 'physical_range':
  // '[-32|31]', 'bit': 21, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // 'deg'}
  int can_rx_high_yaw_angle(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'CAN_RX_CLEAR_FAULTS', 'enum': {0:
  // 'CAN_RX_CLEAR_FAULTS_OFF', 1: 'CAN_RX_CLEAR_FAULTS_ON'}, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]',
  // 'bit': 22, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle2_4f1::Can_rx_clear_faultsType can_rx_clear_faults(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_USE_ANGLE_MISALIGNMENT', 'enum': {0:
  // 'CAN_RX_USE_ANGLE_MISALIGNMENT_OFF', 1:
  // 'CAN_RX_USE_ANGLE_MISALIGNMENT_ON'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle2_4f1::Can_rx_use_angle_misalignmentType can_rx_use_angle_misalignment(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_TURN_SIGNAL_STATUS', 'enum': {0:
  // 'CAN_RX_TURN_SIGNAL_STATUS_OFF', 1: 'CAN_RX_TURN_SIGNAL_STATUS_LEFT', 2:
  // 'CAN_RX_TURN_SIGNAL_STATUS_RIGHT', 3:
  // 'CAN_RX_TURN_SIGNAL_STATUS_INVALID_3'}, 'precision': 1.0, 'len': 2,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 63, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle2_4f1::Can_rx_turn_signal_statusType can_rx_turn_signal_status(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_BLOCKAGE_DISABLE', 'enum': {0:
  // 'CAN_RX_BLOCKAGE_DISABLE_ENABLED', 1: 'CAN_RX_BLOCKAGE_DISABLE_DISABLED'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 54, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Vehicle2_4f1::Can_rx_blockage_disableType can_rx_blockage_disable(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_VEHICLE_SPEED_VALIDITY', 'enum': {0:
  // 'CAN_RX_VEHICLE_SPEED_VALIDITY_INVALID', 1:
  // 'CAN_RX_VEHICLE_SPEED_VALIDITY_VALID'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 61, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle2_4f1::Can_rx_vehicle_speed_validityType can_rx_vehicle_speed_validity(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_MMR_UPSIDE_DOWN', 'enum': {0:
  // 'CAN_RX_MMR_UPSIDE_DOWN_RIGHT_SIDE_UP', 1:
  // 'CAN_RX_MMR_UPSIDE_DOWN_UPSIDE_DOWN'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit':
  // 60, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle2_4f1::Can_rx_mmr_upside_downType can_rx_mmr_upside_down(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_WIPER_STATUS', 'enum': {0:
  // 'CAN_RX_WIPER_STATUS_OFF', 1: 'CAN_RX_WIPER_STATUS_ON'}, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]',
  // 'bit': 57, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle2_4f1::Can_rx_wiper_statusType can_rx_wiper_status(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_RAW_DATA_ENABLE', 'enum': {0:
  // 'CAN_RX_RAW_DATA_ENABLE_FILTERED', 1: 'CAN_RX_RAW_DATA_ENABLE_RAW'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 56, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Vehicle2_4f1::Can_rx_raw_data_enableType can_rx_raw_data_enable(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_RADAR_CMD_RADIATE', 'enum': {0:
  // 'CAN_RX_RADAR_CMD_RADIATE_OFF', 1: 'CAN_RX_RADAR_CMD_RADIATE_ON'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 55, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Vehicle2_4f1::Can_rx_radar_cmd_radiateType can_rx_radar_cmd_radiate(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_GROUPING_MODE', 'enum': {0:
  // 'CAN_RX_GROUPING_MODE_NO_GROUPING', 1:
  // 'CAN_RX_GROUPING_MODE_GROUP_MOVING_ONLY', 2:
  // 'CAN_RX_GROUPING_MODE_GROUP_STATIONARY_ONLY', 3:
  // 'CAN_RX_GROUPING_MODE_GROUP_MOVING_STATIONARY'}, 'precision': 1.0, 'len':
  // 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit':
  // 59, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle2_4f1::Can_rx_grouping_modeType can_rx_grouping_mode(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_MAXIMUM_TRACKS', 'offset': 1.0,
  // 'precision': 1.0, 'len': 6, 'is_signed_var': False, 'physical_range':
  // '[1|64]', 'bit': 53, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_rx_maximum_tracks(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'description': "(+) = to the right from driver's
  // perspective", 'offset': 0.0, 'precision': 0.015625, 'len': 8, 'name':
  // 'CAN_RX_LATERAL_MOUNTING_OFFSET', 'is_signed_var': True, 'physical_range':
  // '[-2|1.984375]', 'bit': 47, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm'}
  double can_rx_lateral_mounting_offset(const std::uint8_t* bytes,
                                        const int32_t length) const;

  // config detail: {'description': '(+) = clockwise', 'offset': 0.0,
  // 'precision': 0.0625, 'len': 8, 'name': 'CAN_RX_ANGLE_MISALIGNMENT',
  // 'is_signed_var': True, 'physical_range': '[-8|7.9375]', 'bit': 39, 'type':
  // 'double', 'order': 'motorola', 'physical_unit': 'deg'}
  double can_rx_angle_misalignment(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'name': 'CAN_RX_SCAN_INDEX_ACK', 'offset': 0.0,
  // 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[0|65535]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // ''}
  int can_rx_scan_index_ack(const std::uint8_t* bytes,
                            const int32_t length) const;
};

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_VEHICLE2_4F1_H_
