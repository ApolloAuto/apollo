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

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_STATUS1_4E0_H_
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_ESR_STATUS1_4E0_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::DelphiESR;

class Esrstatus14e0 : public apollo::drivers::canbus::ProtocolData<DelphiESR> {
 public:
  static const int32_t ID;
  Esrstatus14e0();
  void Parse(const std::uint8_t* bytes, int32_t length,
             DelphiESR* delphi_esr) const override;

 private:
  // config detail: {'name': 'CAN_TX_DSP_TIMESTAMP', 'offset': 0.0, 'precision':
  // 2.0, 'len': 7, 'is_signed_var': False, 'physical_range': '[0|254]', 'bit':
  // 5, 'type': 'double', 'order': 'motorola', 'physical_unit': 'ms'}
  double can_tx_dsp_timestamp(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'CAN_TX_COMM_ERROR', 'offset': 0.0, 'precision':
  // 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
  // 14, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool can_tx_comm_error(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_YAW_RATE_CALC', 'offset': 0.0, 'precision':
  // 0.0625, 'len': 12, 'is_signed_var': True, 'physical_range':
  // '[-128|127.9375]', 'bit': 47, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'deg/s'}
  double can_tx_yaw_rate_calc(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'CAN_TX_VEHICLE_SPEED_CALC', 'offset': 0.0,
  // 'precision': 0.0625, 'len': 11, 'is_signed_var': False, 'physical_range':
  // '[0|127.9375]', 'bit': 50, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm/s'}
  double can_tx_vehicle_speed_calc(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'name': 'CAN_TX_SCAN_INDEX', 'offset': 0.0, 'precision':
  // 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]',
  // 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int can_tx_scan_index(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_TX_ROLLING_COUNT_1', 'offset': 0.0,
  // 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int can_tx_rolling_count_1(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'name': 'CAN_TX_RADIUS_CURVATURE_CALC', 'offset': 0.0,
  // 'precision': 1.0, 'len': 14, 'is_signed_var': True, 'physical_range':
  // '[-8192|8191]', 'bit': 13, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'm'}
  int can_tx_radius_curvature_calc(const std::uint8_t* bytes,
                                   const int32_t length) const;
};

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_ESR_STATUS1_4E0_H_
