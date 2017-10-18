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

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_VEHICLE6_5F5_H_
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_VEHICLE6_5F5_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::DelphiESR;

class Vehicle65f5 : public apollo::drivers::canbus::ProtocolData<DelphiESR> {
 public:
  static const int32_t ID;
  Vehicle65f5();
  void Parse(const std::uint8_t* bytes, int32_t length,
             DelphiESR* delphi_esr) const override;

 private:
  // config detail: {'name': 'CAN_RX_INNER_FUNNEL_OFFSET_RIGHT', 'offset': 0.0,
  // 'precision': 0.1, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-2|10]', 'bit': 39, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm'}
  double can_rx_inner_funnel_offset_right(const std::uint8_t* bytes,
                                          const int32_t length) const;

  // config detail: {'name': 'CAN_RX_INNER_FUNNEL_OFFSET_LEFT', 'offset': 0.0,
  // 'precision': 0.1, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-2|10]', 'bit': 31, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm'}
  double can_rx_inner_funnel_offset_left(const std::uint8_t* bytes,
                                         const int32_t length) const;

  // config detail: {'name': 'CAN_VOLVO_FA_RANGE_MAX_SHORT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit':
  // 'm'}
  int can_volvo_fa_range_max_short(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'name': 'CAN_VOLVO_FA_MIN_VSPEED_SHORT', 'offset': 0.0,
  // 'precision': 0.125, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|20]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit':
  // 'm/s'}
  double can_volvo_fa_min_vspeed_short(const std::uint8_t* bytes,
                                       const int32_t length) const;

  // config detail: {'name': 'CAN_VOLVO_FA_AALIGN_ESTIMATE', 'offset': 0.0,
  // 'precision': 0.0625, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|10]', 'bit': 15, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'deg'}
  double can_volvo_fa_aalign_estimate(const std::uint8_t* bytes,
                                      const int32_t length) const;
};

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_VEHICLE6_5F5_H_
