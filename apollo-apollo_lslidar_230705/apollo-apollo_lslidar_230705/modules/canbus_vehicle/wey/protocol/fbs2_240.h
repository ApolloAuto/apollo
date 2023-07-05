/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include "modules/canbus_vehicle/wey/proto/wey.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace wey {

class Fbs2240 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::Wey> {
 public:
  static const int32_t ID;
  Fbs2240();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Wey* chassis) const override;

 private:
  // config detail: {'description': 'Front left wheel Moving direction',
  // 'enum': {0: 'FLWHEELDIRECTION_INVALID', 1: 'FLWHEELDIRECTION_FORWARD',
  // 2: 'FLWHEELDIRECTION_BACKWARD', 3: 'FLWHEELDIRECTION_STOP'},
  // 'precision': 1.0, 'len': 2, 'name': 'FLWheelDirection',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]',
  // 'bit': 57, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Fbs2_240::FlwheeldirectionType flwheeldirection(const std::uint8_t* bytes,
                                                  const int32_t length) const;

  // config detail: {'description': 'Front right wheel speed', 'offset': 0.0,
  // 'precision': 0.05625, 'len': 13, 'name': 'FRWheelSpd',
  // 'is_signed_var': False, 'physical_range': '[0|299.98125]',
  // 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Km/h'}
  double frwheelspd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Rear left wheel  Moving direction',
  // 'enum': {0: 'RLWHEELDRIVEDIRECTION_INVALID',
  // 1: 'RLWHEELDRIVEDIRECTION_FORWARD', 2: 'RLWHEELDRIVEDIRECTION_BACKWARD',
  // 3: 'RLWHEELDRIVEDIRECTION_STOP'}, 'precision': 1.0, 'len': 2, 'name':
  // 'RLWheelDriveDirection', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|3]', 'bit': 9, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Fbs2_240::RlwheeldrivedirectionType rlwheeldrivedirection(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Rear left wheel speed', 'offset': 0.0,
  // 'precision': 0.05625, 'len': 13, 'name': 'RLWheelSpd',
  // 'is_signed_var': False, 'physical_range': '[0|299.98125]',
  // 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Km/h'}
  double rlwheelspd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Rear right wheel Moving direction',
  // 'enum': {0: 'RRWHEELDIRECTION_INVALID', 1: 'RRWHEELDIRECTION_FORWARD',
  // 2: 'RRWHEELDIRECTION_BACKWARD', 3: 'RRWHEELDIRECTION_STOP'},
  // 'precision': 1.0, 'len': 2, 'name': 'RRWheelDirection',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]'
  // 'bit': 25, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Fbs2_240::RrwheeldirectionType rrwheeldirection(const std::uint8_t* bytes,
                                                  const int32_t length) const;

  // config detail: {'description': 'Rear right wheel speed', 'offset': 0.0,
  // 'precision': 0.05625, 'len': 13, 'name': 'RRWheelSpd',
  // 'is_signed_var': False, 'physical_range': '[0|299.98125]', 'bit': 39,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'Km/h'}
  double rrwheelspd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Current Vehicle speed information',
  // 'offset': 0.0, 'precision': 0.05625, 'len': 13, 'name': 'VehicleSpd',
  // 'is_signed_var': False, 'physical_range': '[0|299.98125]', 'bit': 55,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'Km/h'}
  double vehiclespd(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
