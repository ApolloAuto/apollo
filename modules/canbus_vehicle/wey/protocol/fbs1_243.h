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

class Fbs1243 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::Wey> {
 public:
  static const int32_t ID;
  Fbs1243();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Wey* chassis) const override;

 private:
  // config detail: {'description': 'Longitude acceleration', 'offset': -21.592,
  // 'precision': 0.00098, 'len': 16, 'name': 'LongitudeAcce',
  // 'is_signed_var': False, 'physical_range': '[-21.592|21.592]', 'bit': 7,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s^2'}
  double longitudeacce(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Indicates Lateral Acceleration',
  // 'offset': -21.592, 'precision': 0.00098, 'len': 16, 'name': 'LateralAcce',
  // 'is_signed_var': False, 'physical_range': '[-21.592|21.592]', 'bit': 23,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s^2'}
  double lateralacce(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Vehicle yaw rate', 'offset': -2.093,
  // 'precision': 0.00024, 'len': 16, 'name': 'VehDynYawRate',
  // 'is_signed_var': False, 'physical_range': '[-2.093|2.093]', 'bit': 39,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'rad/s'}
  double vehdynyawrate(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Front left wheel speed', 'offset': 0.0,
  // 'precision': 0.05625, 'len': 13, 'name': 'FLWheelSpd',
  // 'is_signed_var': False, 'physical_range': '[0|299.98125]', 'bit': 55,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'Km/h'}
  double flwheelspd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Front right wheel  Moving direction',
  // 'enum': {0: 'FRWHEELDIRECTION_INVALID', 1: 'FRWHEELDIRECTION_FORWARD',
  // 2: 'FRWHEELDIRECTION_BACKWARD', 3: 'FRWHEELDIRECTION_STOP'},
  // 'precision': 1.0, 'len': 2, 'name': 'FRWheelDirection',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]',
  // 'bit': 57, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Fbs1_243::FrwheeldirectionType frwheeldirection(const std::uint8_t* bytes,
                                                  const int32_t length) const;
};

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
