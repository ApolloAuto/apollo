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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

class Aebwheelimpulse355 : public ::apollo::drivers::canbus::ProtocolData<
                               ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Aebwheelimpulse355();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
  // 'precision': 1.0, 'len': 10, 'name': 'FLImpulse', 'is_signed_var': False,
  // 'physical_range': '[0.0|1023.0]', 'bit': 7, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'bit'}
  double flimpulse(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'FLImpulseValid', 'is_signed_var':
  // False, 'physical_range': '[0.0|1.0]', 'bit': 13, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': 'bit'}
  bool flimpulsevalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'FRImpulse', 'offset': 0.0, 'precision': 1.0,
  // 'len': 10, 'is_signed_var': False, 'physical_range': '[0.0|1023.0]', 'bit':
  // 12, 'type': 'double', 'order': 'motorola', 'physical_unit': 'km/h'}
  double frimpulse(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'FRImpulseValid', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0.0|1.0]', 'bit': 18,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': 'km/h'}
  bool frimpulsevalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
  // 'precision': 1.0, 'len': 10, 'name': 'RLImpulse', 'is_signed_var': False,
  // 'physical_range': '[0.0|1023.0]', 'bit': 17, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'bit'}
  double rlimpulse(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'RLImpulseValid', 'is_signed_var':
  // False, 'physical_range': '[0.0|1.0]', 'bit': 39, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': 'bit'}
  bool rlimpulsevalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'RRImpulse', 'offset': 0.0, 'precision': 1.0,
  // 'len': 10, 'is_signed_var': False, 'physical_range': '[0.0|1023.0]', 'bit':
  // 38, 'type': 'double', 'order': 'motorola', 'physical_unit': 'km/h'}
  double rrimpulse(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'RRImpulseValid', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0.0|1.0]', 'bit': 44,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': 'km/h'}
  bool rrimpulsevalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'AliveCounter', 'offset': 0.0, 'precision': 1.0,
  // 'len': 4, 'is_signed_var': False, 'physical_range': '[0.0|15.0]', 'bit':
  // 51, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  double alivecounter(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum', 'offset': 0.0, 'precision': 1.0, 'len':
  // 8, 'is_signed_var': False, 'physical_range': '[0.0|255.0]', 'bit': 63,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  double checksum(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
