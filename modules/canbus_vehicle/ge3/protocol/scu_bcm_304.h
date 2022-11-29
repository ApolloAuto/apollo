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

class Scubcm304 : public ::apollo::drivers::canbus::ProtocolData<
                      ::apollo::canbus::Ge3> {
 public:
  static const int32_t ID;
  Scubcm304();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ge3* chassis) const override;

 private:
  // config detail: {'description': 'Vehicle reverse status', 'enum': {0:
  // 'BCM_VEHREVERSEST_NORMAL', 1: 'BCM_VEHREVERSEST_REVERSE'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCM_VehReverseSt', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 11, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': '-'}
  Scu_bcm_304::Bcm_vehreversestType bcm_vehreversest(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Right turn lamp status', 'enum': {0:
  // 'BCM_RIGHTTURNLAMPST_INACTIVE', 1: 'BCM_RIGHTTURNLAMPST_ACTIVE'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCM_RightTurnLampSt', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 4, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': '-'}
  Scu_bcm_304::Bcm_rightturnlampstType bcm_rightturnlampst(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Rear fog lamp status', 'enum': {0:
  // 'BCM_REARFOGLAMPST_INACTIVE', 1: 'BCM_REARFOGLAMPST_ACTIVE'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCM_RearFogLampSt', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': '-'}
  Scu_bcm_304::Bcm_rearfoglampstType bcm_rearfoglampst(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Parking lamp status', 'enum': {0:
  // 'BCM_PARKINGLAMPST_INACTIVE', 1: 'BCM_PARKINGLAMPST_ACTIVE'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCM_ParkingLampSt', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Scu_bcm_304::Bcm_parkinglampstType bcm_parkinglampst(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Low beam status', 'enum': {0:
  // 'BCM_LOWBEAMST_INACTIVE', 1: 'BCM_LOWBEAMST_ACTIVE'}, 'precision': 1.0,
  // 'len': 1, 'name': 'BCM_LowBeamSt', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': '-'}
  Scu_bcm_304::Bcm_lowbeamstType bcm_lowbeamst(const std::uint8_t* bytes,
                                               const int32_t length) const;

  // config detail: {'description': 'Left turn lamp status', 'enum': {0:
  // 'BCM_LEFTTURNLAMPST_INACTIVE', 1: 'BCM_LEFTTURNLAMPST_ACTIVE'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCM_LeftTurnLampSt', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': '-'}
  Scu_bcm_304::Bcm_leftturnlampstType bcm_leftturnlampst(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Key status', 'enum': {0: 'BCM_KEYST_OFF',
  // 1: 'BCM_KEYST_ACC', 2: 'BCM_KEYST_ON', 3: 'BCM_KEYST_CRANK'},
  // 'precision': 1.0, 'len': 2, 'name': 'BCM_KeySt', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': '-'}
  Scu_bcm_304::Bcm_keystType bcm_keyst(const std::uint8_t* bytes,
                                       const int32_t length) const;

  // config detail: {'description': 'Horn status', 'enum': {0:
  // 'BCM_HORNST_INACTIVE', 1: 'BCM_HORNST_ACTIVE'}, 'precision': 1.0, 'len': 1,
  // 'name': 'BCM_HornSt', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': '-'}
  Scu_bcm_304::Bcm_hornstType bcm_hornst(const std::uint8_t* bytes,
                                         const int32_t length) const;

  // config detail: {'description': 'High beam status', 'enum': {0:
  // 'BCM_HIGHBEAMST_INACTIVE', 1: 'BCM_HIGHBEAMST_ACTIVE'}, 'precision': 1.0,
  // 'len': 1, 'name': 'BCM_HighBeamSt', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': '-'}
  Scu_bcm_304::Bcm_highbeamstType bcm_highbeamst(const std::uint8_t* bytes,
                                                 const int32_t length) const;

  // config detail: {'description': 'Hazard lamp status', 'enum': {0:
  // 'BCM_HAZARDLAMPST_INACTIVE', 1: 'BCM_HAZARDLAMPST_ACTIVE'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCM_HazardLampSt', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 13, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Scu_bcm_304::Bcm_hazardlampstType bcm_hazardlampst(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Front fog lamp status', 'enum': {0:
  // 'BCM_FRONTFOGLAMPST_INACTIVE', 1: 'BCM_FRONTFOGLAMPST_ACTIVE'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCM_FrontFogLampSt', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': '-'}
  Scu_bcm_304::Bcm_frontfoglampstType bcm_frontfoglampst(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Brake light switch status', 'enum': {0:
  // 'BCM_BRAKELIGHTSWITCHST_INACTIVE', 1: 'BCM_BRAKELIGHTSWITCHST_ACTIVE'},
  // 'precision': 1.0, 'len': 1, 'name': 'BCM_BrakeLightSwitchSt',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit':
  // 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': '-'}
  Scu_bcm_304::Bcm_brakelightswitchstType bcm_brakelightswitchst(
      const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
