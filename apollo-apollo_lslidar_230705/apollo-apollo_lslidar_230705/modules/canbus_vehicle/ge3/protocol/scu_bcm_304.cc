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

#include "modules/canbus_vehicle/ge3/protocol/scu_bcm_304.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

Scubcm304::Scubcm304() {}
const int32_t Scubcm304::ID = 0x304;

void Scubcm304::Parse(const std::uint8_t* bytes, int32_t length,
                      Ge3* chassis) const {
  chassis->mutable_scu_bcm_304()->set_bcm_vehreversest(
      bcm_vehreversest(bytes, length));
  chassis->mutable_scu_bcm_304()->set_bcm_rightturnlampst(
      bcm_rightturnlampst(bytes, length));
  chassis->mutable_scu_bcm_304()->set_bcm_rearfoglampst(
      bcm_rearfoglampst(bytes, length));
  chassis->mutable_scu_bcm_304()->set_bcm_parkinglampst(
      bcm_parkinglampst(bytes, length));
  chassis->mutable_scu_bcm_304()->set_bcm_lowbeamst(
      bcm_lowbeamst(bytes, length));
  chassis->mutable_scu_bcm_304()->set_bcm_leftturnlampst(
      bcm_leftturnlampst(bytes, length));
  chassis->mutable_scu_bcm_304()->set_bcm_keyst(
      bcm_keyst(bytes, length));
  chassis->mutable_scu_bcm_304()->set_bcm_hornst(
      bcm_hornst(bytes, length));
  chassis->mutable_scu_bcm_304()->set_bcm_highbeamst(
      bcm_highbeamst(bytes, length));
  chassis->mutable_scu_bcm_304()->set_bcm_hazardlampst(
      bcm_hazardlampst(bytes, length));
  chassis->mutable_scu_bcm_304()->set_bcm_frontfoglampst(
      bcm_frontfoglampst(bytes, length));
  chassis->mutable_scu_bcm_304()->set_bcm_brakelightswitchst(
      bcm_brakelightswitchst(bytes, length));
}

// config detail: {'description': 'Vehicle reverse status', 'enum': {0:
// 'BCM_VEHREVERSEST_NORMAL', 1: 'BCM_VEHREVERSEST_REVERSE'}, 'precision': 1.0,
// 'len': 1, 'name': 'bcm_vehreversest', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 11, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': '-'}
Scu_bcm_304::Bcm_vehreversestType Scubcm304::bcm_vehreversest(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  Scu_bcm_304::Bcm_vehreversestType ret =
      static_cast<Scu_bcm_304::Bcm_vehreversestType>(x);
  return ret;
}

// config detail: {'description': 'Right turn lamp status', 'enum': {0:
// 'BCM_RIGHTTURNLAMPST_INACTIVE', 1: 'BCM_RIGHTTURNLAMPST_ACTIVE'},
// 'precision': 1.0, 'len': 1, 'name': 'bcm_rightturnlampst', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 4, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': '-'}
Scu_bcm_304::Bcm_rightturnlampstType Scubcm304::bcm_rightturnlampst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 1);

  Scu_bcm_304::Bcm_rightturnlampstType ret =
      static_cast<Scu_bcm_304::Bcm_rightturnlampstType>(x);
  return ret;
}

// config detail: {'description': 'Rear fog lamp status', 'enum': {0:
// 'BCM_REARFOGLAMPST_INACTIVE', 1: 'BCM_REARFOGLAMPST_ACTIVE'},
// 'precision': 1.0, 'len': 1, 'name': 'bcm_rearfoglampst', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': '-'}
Scu_bcm_304::Bcm_rearfoglampstType Scubcm304::bcm_rearfoglampst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Scu_bcm_304::Bcm_rearfoglampstType ret =
      static_cast<Scu_bcm_304::Bcm_rearfoglampstType>(x);
  return ret;
}

// config detail: {'description': 'Parking lamp status', 'enum': {0:
// 'BCM_PARKINGLAMPST_INACTIVE', 1: 'BCM_PARKINGLAMPST_ACTIVE'},
// 'precision': 1.0, 'len': 1, 'name': 'bcm_parkinglampst', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Scu_bcm_304::Bcm_parkinglampstType Scubcm304::bcm_parkinglampst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  Scu_bcm_304::Bcm_parkinglampstType ret =
      static_cast<Scu_bcm_304::Bcm_parkinglampstType>(x);
  return ret;
}

// config detail: {'description': 'Low beam status', 'enum': {0:
// 'BCM_LOWBEAMST_INACTIVE', 1: 'BCM_LOWBEAMST_ACTIVE'}, 'precision': 1.0,
// 'len': 1, 'name': 'bcm_lowbeamst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': '-'}
Scu_bcm_304::Bcm_lowbeamstType Scubcm304::bcm_lowbeamst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  Scu_bcm_304::Bcm_lowbeamstType ret =
      static_cast<Scu_bcm_304::Bcm_lowbeamstType>(x);
  return ret;
}

// config detail: {'description': 'Left turn lamp status', 'enum': {0:
// 'BCM_LEFTTURNLAMPST_INACTIVE', 1: 'BCM_LEFTTURNLAMPST_ACTIVE'},
// 'precision': 1.0, 'len': 1, 'name': 'bcm_leftturnlampst', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': '-'}
Scu_bcm_304::Bcm_leftturnlampstType Scubcm304::bcm_leftturnlampst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  Scu_bcm_304::Bcm_leftturnlampstType ret =
      static_cast<Scu_bcm_304::Bcm_leftturnlampstType>(x);
  return ret;
}

// config detail: {'description': 'Key status', 'enum': {0: 'BCM_KEYST_OFF', 1:
// 'BCM_KEYST_ACC', 2: 'BCM_KEYST_ON', 3: 'BCM_KEYST_CRANK'}, 'precision': 1.0,
// 'len': 2, 'name': 'bcm_keyst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': '-'}
Scu_bcm_304::Bcm_keystType Scubcm304::bcm_keyst(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Scu_bcm_304::Bcm_keystType ret = static_cast<Scu_bcm_304::Bcm_keystType>(x);
  return ret;
}

// config detail: {'description': 'Horn status', 'enum': {0:
// 'BCM_HORNST_INACTIVE', 1: 'BCM_HORNST_ACTIVE'}, 'precision': 1.0, 'len': 1,
// 'name': 'bcm_hornst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': '-'}
Scu_bcm_304::Bcm_hornstType Scubcm304::bcm_hornst(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  Scu_bcm_304::Bcm_hornstType ret = static_cast<Scu_bcm_304::Bcm_hornstType>(x);
  return ret;
}

// config detail: {'description': 'High beam status', 'enum': {0:
// 'BCM_HIGHBEAMST_INACTIVE', 1: 'BCM_HIGHBEAMST_ACTIVE'}, 'precision': 1.0,
// 'len': 1, 'name': 'bcm_highbeamst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': '-'}
Scu_bcm_304::Bcm_highbeamstType Scubcm304::bcm_highbeamst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  Scu_bcm_304::Bcm_highbeamstType ret =
      static_cast<Scu_bcm_304::Bcm_highbeamstType>(x);
  return ret;
}

// config detail: {'description': 'Hazard lamp status', 'enum': {0:
// 'BCM_HAZARDLAMPST_INACTIVE', 1: 'BCM_HAZARDLAMPST_ACTIVE'}, 'precision': 1.0,
// 'len': 1, 'name': 'bcm_hazardlampst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 13, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_bcm_304::Bcm_hazardlampstType Scubcm304::bcm_hazardlampst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  Scu_bcm_304::Bcm_hazardlampstType ret =
      static_cast<Scu_bcm_304::Bcm_hazardlampstType>(x);
  return ret;
}

// config detail: {'description': 'Front fog lamp status', 'enum': {0:
// 'BCM_FRONTFOGLAMPST_INACTIVE', 1: 'BCM_FRONTFOGLAMPST_ACTIVE'},
// 'precision': 1.0, 'len': 1, 'name': 'bcm_frontfoglampst', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': '-'}
Scu_bcm_304::Bcm_frontfoglampstType Scubcm304::bcm_frontfoglampst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  Scu_bcm_304::Bcm_frontfoglampstType ret =
      static_cast<Scu_bcm_304::Bcm_frontfoglampstType>(x);
  return ret;
}

// config detail: {'description': 'Brake light switch status', 'enum': {0:
// 'BCM_BRAKELIGHTSWITCHST_INACTIVE', 1: 'BCM_BRAKELIGHTSWITCHST_ACTIVE'},
// 'precision': 1.0, 'len': 1, 'name': 'bcm_brakelightswitchst',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 14,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': '-'}
Scu_bcm_304::Bcm_brakelightswitchstType Scubcm304::bcm_brakelightswitchst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  Scu_bcm_304::Bcm_brakelightswitchstType ret =
      static_cast<Scu_bcm_304::Bcm_brakelightswitchstType>(x);
  return ret;
}
}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
