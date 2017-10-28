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

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_VEHICLE5_5F4_H_
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_VEHICLE5_5F4_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::DelphiESR;

class Vehicle55f4 : public apollo::drivers::canbus::ProtocolData<DelphiESR> {
 public:
  static const int32_t ID;
  Vehicle55f4();
  void Parse(const std::uint8_t* bytes, int32_t length,
             DelphiESR* delphi_esr) const override;

 private:
  // config detail: {'name': 'CAN_RX_YAW_RATE_BIAS_SHIFT', 'enum': {0:
  // 'CAN_RX_YAW_RATE_BIAS_SHIFT_NO_DETECT', 1:
  // 'CAN_RX_YAW_RATE_BIAS_SHIFT_DETECT'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit':
  // 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Vehicle5_5f4::Can_rx_yaw_rate_bias_shiftType can_rx_yaw_rate_bias_shift(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CAN_RX_STEERING_GEAR_RATIO', 'offset': 0.0,
  // 'precision': 0.125, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|31.875]', 'bit': 63, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': ''}
  double can_rx_steering_gear_ratio(const std::uint8_t* bytes,
                                    const int32_t length) const;

  // config detail: {'name': 'CAN_RX_WHEELBASE', 'offset': 200.0, 'precision':
  // 2.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[200|710]',
  // 'bit': 55, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
  double can_rx_wheelbase(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'name': 'CAN_RX_DISTANCE_REAR_AXLE', 'offset': 200.0,
  // 'precision': 2.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[200|710]', 'bit': 47, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'cm'}
  double can_rx_distance_rear_axle(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'name': 'CAN_RX_CW_BLOCKAGE_THRESHOLD', 'offset': 0.0,
  // 'precision': 0.0078125, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|1.9921875]', 'bit': 39, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': ''}
  double can_rx_cw_blockage_threshold(const std::uint8_t* bytes,
                                      const int32_t length) const;

  // config detail: {'name': 'CAN_RX_FUNNEL_OFFSET_RIGHT', 'offset': 0.0,
  // 'precision': 0.1, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-2|10]', 'bit': 31, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm'}
  double can_rx_funnel_offset_right(const std::uint8_t* bytes,
                                    const int32_t length) const;

  // config detail: {'name': 'CAN_RX_FUNNEL_OFFSET_LEFT', 'offset': 0.0,
  // 'precision': 0.1, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-2|10]', 'bit': 23, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm'}
  double can_rx_funnel_offset_left(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'name': 'CAN_RX_BEAMWIDTH_VERT', 'offset': 0.0,
  // 'precision': 0.0625, 'len': 7, 'is_signed_var': False, 'physical_range':
  // '[0|6]', 'bit': 14, 'type': 'double', 'order': 'motorola', 'physical_unit':
  // 'deg'}
  double can_rx_beamwidth_vert(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'name': 'CAN_RX_OVERSTEER_UNDERSTEER', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-128|127]', 'bit': 7, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '%'}
  int can_rx_oversteer_understeer(const std::uint8_t* bytes,
                                  const int32_t length) const;
};

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_VEHICLE5_5F4_H_
