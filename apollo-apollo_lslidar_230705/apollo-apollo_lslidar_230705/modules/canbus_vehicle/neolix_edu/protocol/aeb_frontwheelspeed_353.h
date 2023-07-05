/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/neolix_edu/proto/neolix_edu.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

class Aebfrontwheelspeed353 : public ::apollo::drivers::canbus::ProtocolData<
                                  ::apollo::canbus::Neolix_edu> {
 public:
  static const int32_t ID;
  Aebfrontwheelspeed353();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Neolix_edu* chassis) const override;

 private:
  // config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'VehicleSpeedValid', 'is_signed_var':
  // False, 'physical_range': '[0.0|1.0]', 'bit': 7, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': 'bit'}
  bool vehiclespeedvalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VehicleSpeed', 'offset': 0.0, 'precision':
  // 0.05625, 'len': 13, 'is_signed_var': False, 'physical_range':
  // '[0.0|460.69]', 'bit': 4, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'Km/h'}
  double vehiclespeed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Invalid;0x1:D;0x2:N;0x3:R', 'offset':
  // 0.0, 'precision': 1.0, 'len': 2, 'name': 'VehicleRealDirect',
  // 'is_signed_var': False, 'physical_range': '[0.0|3.0]', 'bit': 6, 'type':
  // 'double', 'order': 'motorola', 'physical_unit': 'bit'}
  double vehiclerealdirect(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'WheelSpeed_FL_Valid', 'is_signed_var':
  // False, 'physical_range': '[0.0|1.0]', 'bit': 23, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': 'bit'}
  bool wheelspeed_fl_valid(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'name': 'WheelSpeed_FL', 'offset': 0.0, 'precision': 0.01,
  // 'len': 15, 'is_signed_var': False, 'physical_range': '[0.0|327.67]', 'bit':
  // 22, 'type': 'double', 'order': 'motorola', 'physical_unit': 'km/h'}
  double wheelspeed_fl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'WheelSpeed_FR_Valid', 'is_signed_var':
  // False, 'physical_range': '[0.0|1.0]', 'bit': 39, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': 'bit'}
  bool wheelspeed_fr_valid(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'name': 'WheelSpeed_FR', 'offset': 0.0, 'precision': 0.01,
  // 'len': 15, 'is_signed_var': False, 'physical_range': '[0.0|327.67]', 'bit':
  // 38, 'type': 'double', 'order': 'motorola', 'physical_unit': 'km/h'}
  double wheelspeed_fr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Invalid;0x1:D;0x2:N;0x3:R', 'offset':
  // 0.0, 'precision': 1.0, 'len': 2, 'name': 'WheelSpeed_FL_Direct',
  // 'is_signed_var': False, 'physical_range': '[0.0|3.0]', 'bit': 53, 'type':
  // 'double', 'order': 'motorola', 'physical_unit': 'bit'}
  double wheelspeed_fl_direct(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'description': '0x0:Invalid;0x1:D;0x2:N;0x3:R', 'offset':
  // 0.0, 'precision': 1.0, 'len': 2, 'name': 'WheelSpeed_FR_Direct',
  // 'is_signed_var': False, 'physical_range': '[0.0|3.0]', 'bit': 55, 'type':
  // 'double', 'order': 'motorola', 'physical_unit': 'bit'}
  double wheelspeed_fr_direct(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'name': 'AliveCounter_Front', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0.0|15.0]', 'bit': 51, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': ''}
  double alivecounter_front(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'Checksum_Front', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0.0|255.0]', 'bit':
  // 63, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  double checksum_front(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
