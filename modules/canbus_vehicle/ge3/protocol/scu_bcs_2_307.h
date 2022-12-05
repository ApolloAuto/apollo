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

class Scubcs2307 : public ::apollo::drivers::canbus::ProtocolData<
                       ::apollo::canbus::Ge3> {
 public:
  static const int32_t ID;
  Scubcs2307();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ge3* chassis) const override;

 private:
  // config detail: {'description': 'Vehicle speed valid data', 'enum': {0:
  // 'BCS_VEHSPDVD_INVALID', 1: 'BCS_VEHSPDVD_VALID'}, 'precision': 1.0, 'len':
  // 1, 'name': 'BCS_VehSpdVD', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 40, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': '-'}
  Scu_bcs_2_307::Bcs_vehspdvdType bcs_vehspdvd(const std::uint8_t* bytes,
                                               const int32_t length) const;

  // config detail: {'description': 'Yaw rate', 'offset': -2.2243, 'precision':
  // 0.0021326, 'len': 12, 'name': 'BCS_YawRate', 'is_signed_var': False,
  // 'physical_range': '[-2.2243|2.2243]', 'bit': 55, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'rad/s'}
  double bcs_yawrate(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Vehicle speed', 'offset': 0.0, 'precision':
  // 0.05625, 'len': 13, 'name': 'BCS_VehSpd', 'is_signed_var': False,
  // 'physical_range': '[0|240]', 'bit': 39, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'km/h'}
  double bcs_vehspd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Vehicle longitudinal acceleration',
  // 'offset': -21.593, 'precision': 0.027126736, 'len': 12, 'name':
  // 'BCS_VehLongAccel', 'is_signed_var': False, 'physical_range':
  // '[-21.593|21.593]', 'bit': 23, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'm/s^2'}
  double bcs_vehlongaccel(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'description': 'Vehicle lateral acceleration', 'offset':
  // -21.593, 'precision': 0.027126736, 'len': 12, 'name': 'BCS_VehLatAccel',
  // 'is_signed_var': False, 'physical_range': '[-21.593|21.593]', 'bit': 7,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s^2'}
  double bcs_vehlataccel(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
