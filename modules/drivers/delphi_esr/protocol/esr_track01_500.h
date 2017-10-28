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

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_TRACK01_500_H_
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_TRACK01_500_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::DelphiESR;

class Esrtrack01500 : public apollo::drivers::canbus::ProtocolData<DelphiESR> {
 public:
  static const int32_t ID;
  Esrtrack01500();
  void Parse(const std::uint8_t* bytes, int32_t length,
             DelphiESR* delphi_esr) const override;

 private:
  // config detail: {'name': 'CAN_TX_TRACK_GROUPING_CHANGED', 'enum': {0:
  // 'CAN_TX_TRACK_GROUPING_CHANGED_GROUPINGUNCHANGED', 1:
  // 'CAN_TX_TRACK_GROUPING_CHANGED_GROUPINGCHANGED'}, 'precision': 1.0, 'len':
  // 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_track01_500::Can_tx_track_grouping_changedType
  can_tx_track_grouping_changed(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'name': 'CAN_TX_TRACK_ONCOMING', 'enum': {0:
  // 'CAN_TX_TRACK_ONCOMING_NOTONCOMING', 1: 'CAN_TX_TRACK_ONCOMING_ONCOMING'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|0]', 'bit': 0, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Esr_track01_500::Can_tx_track_oncomingType can_tx_track_oncoming(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_TRACK_LAT_RATE', 'offset': 0.0,
  // 'precision': 0.25, 'len': 6, 'is_signed_var': True, 'physical_range':
  // '[-8|7.75]', 'bit': 7, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': ''}
  double can_tx_track_lat_rate(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'name': 'CAN_TX_TRACK_BRIDGE_OBJECT', 'enum': {0:
  // 'CAN_TX_TRACK_BRIDGE_OBJECT_NOT_BRIDGE', 1:
  // 'CAN_TX_TRACK_BRIDGE_OBJECT_BRIDGE'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_track01_500::Can_tx_track_bridge_objectType can_tx_track_bridge_object(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_TRACK_WIDTH', 'offset': 0.0, 'precision':
  // 0.5, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|7.5]', 'bit':
  // 37, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
  double can_tx_track_width(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'CAN_TX_TRACK_STATUS', 'enum': {0:
  // 'CAN_TX_TRACK_STATUS_NO_TARGET', 1: 'CAN_TX_TRACK_STATUS_NEW_TARGET', 2:
  // 'CAN_TX_TRACK_STATUS_NEW_UPDATED_TARGET', 3:
  // 'CAN_TX_TRACK_STATUS_UPDATED_TARGET', 4:
  // 'CAN_TX_TRACK_STATUS_COASTED_TARGET', 5:
  // 'CAN_TX_TRACK_STATUS_MERGED_TARGET', 6:
  // 'CAN_TX_TRACK_STATUS_INVALID_COASTED_TARGET', 7:
  // 'CAN_TX_TRACK_STATUS_NEW_COASTED_TARGET'}, 'precision': 1.0, 'len': 3,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit':
  // 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_track01_500::Can_tx_track_statusType can_tx_track_status(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_TRACK_ROLLING_COUNT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 38, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool can_tx_track_rolling_count(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'name': 'CAN_TX_TRACK_RANGE_RATE', 'offset': 0.0,
  // 'precision': 0.01, 'len': 14, 'is_signed_var': True, 'physical_range':
  // '[-81.92|81.91]', 'bit': 53, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm/s'}
  double can_tx_track_range_rate(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'name': 'CAN_TX_TRACK_RANGE_ACCEL', 'offset': 0.0,
  // 'precision': 0.05, 'len': 10, 'is_signed_var': True, 'physical_range':
  // '[-25.6|25.55]', 'bit': 33, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm/s/s'}
  double can_tx_track_range_accel(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'name': 'CAN_TX_TRACK_RANGE', 'offset': 0.0, 'precision':
  // 0.1, 'len': 11, 'is_signed_var': False, 'physical_range': '[0|204.7]',
  // 'bit': 18, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
  double can_tx_track_range(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'CAN_TX_TRACK_MED_RANGE_MODE', 'enum': {0:
  // 'CAN_TX_TRACK_MED_RANGE_MODE_NO_MR_LR_UPDATE', 1:
  // 'CAN_TX_TRACK_MED_RANGE_MODE_MR_UPDATE_ONLY', 2:
  // 'CAN_TX_TRACK_MED_RANGE_MODE_LR_UPDATE_ONLY', 3:
  // 'CAN_TX_TRACK_MED_RANGE_MODE_BOTH_MR_LR_UPDATE'}, 'precision': 1.0, 'len':
  // 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit':
  // 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esr_track01_500::Can_tx_track_med_range_modeType can_tx_track_med_range_mode(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_TRACK_ANGLE', 'offset': 0.0, 'precision':
  // 0.1, 'len': 10, 'is_signed_var': True, 'physical_range': '[-51.2|51.1]',
  // 'bit': 12, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
  double can_tx_track_angle(const std::uint8_t* bytes,
                            const int32_t length) const;
};

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_ESR_TRACK01_500_H_
