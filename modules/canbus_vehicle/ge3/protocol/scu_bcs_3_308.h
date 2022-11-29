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

#include "modules/canbus_vehicle/ge3/proto/ge3.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ge3 {

class Scubcs3308 : public ::apollo::drivers::canbus::ProtocolData<
                       ::apollo::canbus::Ge3> {
 public:
  static const int32_t ID;
  Scubcs3308();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ge3* chassis) const override;

 private:
  // config detail: {'description': 'Rear right wheel speed valid data', 'enum':
  // {0: 'BCS_RRWHEELSPDVD_INVALID', 1: 'BCS_RRWHEELSPDVD_VALID'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCS_RRWheelSpdVD', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 57, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': '-'}
  Scu_bcs_3_308::Bcs_rrwheelspdvdType bcs_rrwheelspdvd(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Rear right wheel speed direction valid
  // data', 'enum': {0: 'BCS_RRWHEELDIRECTIONVD_INVALID', 1:
  // 'BCS_RRWHEELDIRECTIONVD_VALID'}, 'precision': 1.0, 'len': 1, 'name':
  // 'BCS_RRWheelDirectionVD', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 58, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': '-'}
  Scu_bcs_3_308::Bcs_rrwheeldirectionvdType bcs_rrwheeldirectionvd(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Rear left wheel speed valid data', 'enum':
  // {0: 'BCS_RLWHEELSPDVD_INVALID', 1: 'BCS_RLWHEELSPDVD_VALID'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCS_RLWheelSpdVD', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 41, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': '-'}
  Scu_bcs_3_308::Bcs_rlwheelspdvdType bcs_rlwheelspdvd(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Rear left wheel speed direction valid
  // data', 'enum': {0: 'BCS_RLWHEELDIRECTIONVD_INVALID', 1:
  // 'BCS_RLWHEELDIRECTIONVD_VALID'}, 'precision': 1.0, 'len': 1, 'name':
  // 'BCS_RLWheelDirectionVD', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 42, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': '-'}
  Scu_bcs_3_308::Bcs_rlwheeldirectionvdType bcs_rlwheeldirectionvd(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Front right wheel speed valid data',
  // 'enum': {0: 'BCS_FRWHEELSPDVD_INVALID', 1: 'BCS_FRWHEELSPDVD_VALID'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCS_FRWheelSpdVD', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 25, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': '-'}
  Scu_bcs_3_308::Bcs_frwheelspdvdType bcs_frwheelspdvd(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Front right wheel speed direction valid
  // data', 'enum': {0: 'BCS_FRWHEELDIRECTIONVD_INVALID', 1:
  // 'BCS_FRWHEELDIRECTIONVD_VALID'}, 'precision': 1.0, 'len': 1, 'name':
  // 'BCS_FRWheelDirectionVD', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 26, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': '-'}
  Scu_bcs_3_308::Bcs_frwheeldirectionvdType bcs_frwheeldirectionvd(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Front left wheel speed valid data', 'enum':
  // {0: 'BCS_FLWHEELSPDVD_INVALID', 1: 'BCS_FLWHEELSPDVD_VALID'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCS_FLWheelSpdVD', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': '-'}
  Scu_bcs_3_308::Bcs_flwheelspdvdType bcs_flwheelspdvd(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Front left wheel speed direction valid
  // data', 'enum': {0: 'BCS_FLWHEELDIRECTIONVD_INVALID', 1:
  // 'BCS_FLWHEELDIRECTIONVD_VALID'}, 'precision': 1.0, 'len': 1, 'name':
  // 'BCS_FLWheelDirectionVD', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 10, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': '-'}
  Scu_bcs_3_308::Bcs_flwheeldirectionvdType bcs_flwheeldirectionvd(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Rear right wheel speed', 'offset': 0.0,
  // 'precision': 0.05625, 'len': 13, 'name': 'BCS_RRWheelSpd', 'is_signed_var':
  // False, 'physical_range': '[0|240]', 'bit': 55, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'km/h'}
  double bcs_rrwheelspd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Rear right wheel speed direction', 'enum':
  // {0: 'BCS_RRWHEELDIRECTION_FORWARD', 1: 'BCS_RRWHEELDIRECTION_BACKWARD'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCS_RRWheelDirection',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit':
  // 56, 'type': 'enum', 'order': 'motorola', 'physical_unit': '-'}
  Scu_bcs_3_308::Bcs_rrwheeldirectionType bcs_rrwheeldirection(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Rear left wheel speed', 'offset': 0.0,
  // 'precision': 0.05625, 'len': 13, 'name': 'BCS_RLWheelSpd', 'is_signed_var':
  // False, 'physical_range': '[0|240]', 'bit': 39, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'km/h'}
  double bcs_rlwheelspd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Rear left wheel speed direction', 'enum':
  // {0: 'BCS_RLWHEELDIRECTION_FORWARD', 1: 'BCS_RLWHEELDIRECTION_BACKWARD'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCS_RLWheelDirection',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit':
  // 40, 'type': 'enum', 'order': 'motorola', 'physical_unit': '-'}
  Scu_bcs_3_308::Bcs_rlwheeldirectionType bcs_rlwheeldirection(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Front right wheel speed', 'offset': 0.0,
  // 'precision': 0.05625, 'len': 13, 'name': 'BCS_FRWheelSpd', 'is_signed_var':
  // False, 'physical_range': '[0|240]', 'bit': 23, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'km/h'}
  double bcs_frwheelspd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Front right wheel speed direction', 'enum':
  // {0: 'BCS_FRWHEELDIRECTION_FORWARD', 1: 'BCS_FRWHEELDIRECTION_BACKWARD'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCS_FRWheelDirection',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit':
  // 24, 'type': 'enum', 'order': 'motorola', 'physical_unit': '-'}
  Scu_bcs_3_308::Bcs_frwheeldirectionType bcs_frwheeldirection(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Front left wheel speed', 'offset': 0.0,
  // 'precision': 0.05625, 'len': 13, 'name': 'BCS_FLWheelSpd', 'is_signed_var':
  // False, 'physical_range': '[0|240]', 'bit': 7, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'km/h'}
  double bcs_flwheelspd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Front left wheel speed direction', 'enum':
  // {0: 'BCS_FLWHEELDIRECTION_FORWARD', 1: 'BCS_FLWHEELDIRECTION_BACKWARD'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCS_FLWheelDirection',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': '-'}
  Scu_bcs_3_308::Bcs_flwheeldirectionType bcs_flwheeldirection(
      const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
