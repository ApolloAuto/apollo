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

#include "modules/canbus/vehicle/ge3/protocol/scu_vcu_1_312.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

Scuvcu1312::Scuvcu1312() {}
const int32_t Scuvcu1312::ID = 0x312;

void Scuvcu1312::Parse(const std::uint8_t* bytes, int32_t length,
                       ChassisDetail* chassis) const {
  chassis->mutable_ge3()->mutable_scu_vcu_1_312()->set_vcu_elcsysfault(
      vcu_elcsysfault(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_1_312()->set_vcu_brkpedst(
      vcu_brkpedst(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_1_312()->set_vcu_intidx(
      vcu_intidx(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_1_312()->set_vcu_gearintidx(
      vcu_gearintidx(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_1_312()->set_vcu_geardrvmode(
      vcu_geardrvmode(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_1_312()->set_vcu_accpedact(
      vcu_accpedact(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_1_312()->set_vcu_brkpedpst(
      vcu_brkpedpst(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_1_312()->set_vcu_vehrng(
      vcu_vehrng(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_1_312()->set_vcu_accpedpst(
      vcu_accpedpst(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_1_312()->set_vcu_vehrdyst(
      vcu_vehrdyst(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_1_312()->set_vcu_faultst(
      vcu_faultst(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_1_312()->set_vcu_drvmode(
      vcu_drvmode(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_1_312()->set_vcu_gearpst(
      vcu_gearpst(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_1_312()->set_vcu_gearfaultst(
      vcu_gearfaultst(bytes, length));
  chassis->mutable_ge3()->mutable_scu_vcu_1_312()->set_vcu_gearact(
      vcu_gearact(bytes, length));
  // newcode
  chassis->mutable_check_response()->set_is_vcu_online(
      vcu_drvmode(bytes, length) == 3);
}

// config detail: {'description': 'Gear fault status', 'enum': {0:
// 'VCU_ELCSYSFAULT_NORMAL', 1: 'VCU_ELCSYSFAULT_FAULT'}, 'precision': 1.0,
// 'len': 1, 'name': 'vcu_elcsysfault', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 49, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_vcu_1_312::Vcu_elcsysfaultType Scuvcu1312::vcu_elcsysfault(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(1, 1);

  Scu_vcu_1_312::Vcu_elcsysfaultType ret =
      static_cast<Scu_vcu_1_312::Vcu_elcsysfaultType>(x);
  return ret;
}

// config detail: {'description': 'Brake pedal position', 'enum': {0:
// 'VCU_BRKPEDST_UNPRESSED', 1: 'VCU_BRKPEDST_PRESSED'}, 'precision': 1.0,
// 'len': 1, 'name': 'vcu_brkpedst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 48, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_vcu_1_312::Vcu_brkpedstType Scuvcu1312::vcu_brkpedst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 1);

  Scu_vcu_1_312::Vcu_brkpedstType ret =
      static_cast<Scu_vcu_1_312::Vcu_brkpedstType>(x);
  return ret;
}

// config detail: {'description': 'VCU interrupt index', 'enum': {0:
// 'VCU_INTIDX_NOINT', 1: 'VCU_INTIDX_OVERFLOW', 2: 'VCU_INTIDX_TIMEOUT', 3:
// 'VCU_INTIDX_ACCPEDINT', 4: 'VCU_INTIDX_BRKPEDINT', 5: 'VCU_INTIDX_GEARINT'},
// 'precision': 1.0, 'len': 3, 'name': 'vcu_intidx', 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 58, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Scu_vcu_1_312::Vcu_intidxType Scuvcu1312::vcu_intidx(const std::uint8_t* bytes,
                                                     int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 3);

  Scu_vcu_1_312::Vcu_intidxType ret =
      static_cast<Scu_vcu_1_312::Vcu_intidxType>(x);
  return ret;
}

// config detail: {'description': 'Gear interrupt index', 'enum': {0:
// 'VCU_GEARINTIDX_NOINT', 1: 'VCU_GEARINTIDX_OVERFLOW', 2:
// 'VCU_GEARINTIDX_TIMEOUT'}, 'precision': 1.0, 'len': 3, 'name':
// 'vcu_gearintidx', 'is_signed_var': False, 'offset': 0.0, 'physical_range':
// '[0|7]', 'bit': 61, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Scu_vcu_1_312::Vcu_gearintidxType Scuvcu1312::vcu_gearintidx(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(3, 3);

  Scu_vcu_1_312::Vcu_gearintidxType ret =
      static_cast<Scu_vcu_1_312::Vcu_gearintidxType>(x);
  return ret;
}

// config detail: {'description': 'VCU Gear drive mode', 'enum': {0:
// 'VCU_GEARDRVMODE_INVALID', 1: 'VCU_GEARDRVMODE_MANUAL', 2:
// 'VCU_GEARDRVMODE_INTERRUPT', 3: 'VCU_GEARDRVMODE_AUTO'}, 'precision': 1.0,
// 'len': 2, 'name': 'vcu_geardrvmode', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|3]', 'bit': 63, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_vcu_1_312::Vcu_geardrvmodeType Scuvcu1312::vcu_geardrvmode(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(6, 2);

  Scu_vcu_1_312::Vcu_geardrvmodeType ret =
      static_cast<Scu_vcu_1_312::Vcu_geardrvmodeType>(x);
  return ret;
}

// config detail: {'description': 'Actual acceleration pedal position',
// 'offset': 0.0, 'precision': 0.05, 'len': 12, 'name': 'vcu_accpedact',
// 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 47, 'type':
// 'double', 'order': 'motorola', 'physical_unit': '%'}
double Scuvcu1312::vcu_accpedact(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.050000;
  return ret;
}

// config detail: {'description': 'Brake pedal position', 'offset': 0.0,
// 'precision': 0.392, 'len': 8, 'name': 'vcu_brkpedpst', 'is_signed_var':
// False, 'physical_range': '[0|99.96]', 'bit': 39, 'type': 'double', 'order':
// 'motorola', 'physical_unit': '%'}
double Scuvcu1312::vcu_brkpedpst(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.392000;
  return ret;
}

// config detail: {'description': 'Veh range', 'offset': 0.0, 'precision': 1.0,
// 'len': 10, 'name': 'vcu_vehrng', 'is_signed_var': False, 'physical_range':
// '[0|1000]', 'bit': 9, 'type': 'int', 'order': 'motorola', 'physical_unit':
// 'km'}
int Scuvcu1312::vcu_vehrng(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 2);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'description': 'Actual acceleration pedal position',
// 'offset': 0.0, 'precision': 0.392, 'len': 8, 'name': 'vcu_accpedpst',
// 'is_signed_var': False, 'physical_range': '[0|99.96]', 'bit': 31, 'type':
// 'double', 'order': 'motorola', 'physical_unit': '%'}
double Scuvcu1312::vcu_accpedpst(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.392000;
  return ret;
}

// config detail: {'description': 'HEV system ready status', 'enum': {0:
// 'VCU_VEHRDYST_NOTREADY', 1: 'VCU_VEHRDYST_READY'}, 'precision': 1.0, 'len':
// 1, 'name': 'vcu_vehrdyst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_vcu_1_312::Vcu_vehrdystType Scuvcu1312::vcu_vehrdyst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Scu_vcu_1_312::Vcu_vehrdystType ret =
      static_cast<Scu_vcu_1_312::Vcu_vehrdystType>(x);
  return ret;
}

// config detail: {'description': 'VCU fault status', 'enum': {0:
// 'VCU_FAULTST_NORMAL', 1: 'VCU_FAULTST_DERATE', 2: 'VCU_FAULTST_RSV1', 3:
// 'VCU_FAULTST_RSV2', 4: 'VCU_FAULTST_RSV3', 5: 'VCU_FAULTST_FAULT'},
// 'precision': 1.0, 'len': 4, 'name': 'vcu_faultst', 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 5, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Scu_vcu_1_312::Vcu_faultstType Scuvcu1312::vcu_faultst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 4);

  Scu_vcu_1_312::Vcu_faultstType ret =
      static_cast<Scu_vcu_1_312::Vcu_faultstType>(x);
  return ret;
}

// config detail: {'description': 'VCU drive mode', 'enum': {0:
// 'VCU_DRVMODE_INVALID', 1: 'VCU_DRVMODE_MANUAL', 2: 'VCU_DRVMODE_INTERRUPT',
// 3: 'VCU_DRVMODE_AUTO'}, 'precision': 1.0, 'len': 2, 'name': 'vcu_drvmode',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Scu_vcu_1_312::Vcu_drvmodeType Scuvcu1312::vcu_drvmode(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Scu_vcu_1_312::Vcu_drvmodeType ret =
      static_cast<Scu_vcu_1_312::Vcu_drvmodeType>(x);
  return ret;
}

// config detail: {'description': 'Gear lever position', 'enum': {0:
// 'VCU_GEARPST_INVALID', 1: 'VCU_GEARPST_DRIVE', 2: 'VCU_GEARPST_NEUTRAL', 3:
// 'VCU_GEARPST_REVERSE', 4: 'VCU_GEARPST_PARK'}, 'precision': 1.0, 'len': 3,
// 'name': 'vcu_gearpst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|7]', 'bit': 12, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_vcu_1_312::Vcu_gearpstType Scuvcu1312::vcu_gearpst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 3);

  Scu_vcu_1_312::Vcu_gearpstType ret =
      static_cast<Scu_vcu_1_312::Vcu_gearpstType>(x);
  return ret;
}

// config detail: {'description': 'Gear fault status', 'enum': {0:
// 'VCU_GEARFAULTST_NORMAL', 1: 'VCU_GEARFAULTST_FAULT'}, 'precision': 1.0,
// 'len': 1, 'name': 'vcu_gearfaultst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 1, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_vcu_1_312::Vcu_gearfaultstType Scuvcu1312::vcu_gearfaultst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  Scu_vcu_1_312::Vcu_gearfaultstType ret =
      static_cast<Scu_vcu_1_312::Vcu_gearfaultstType>(x);
  return ret;
}

// config detail: {'description': 'Actual gear', 'enum': {0:
// 'VCU_GEARACT_INVALID', 1: 'VCU_GEARACT_DRIVE', 2: 'VCU_GEARACT_NEUTRAL', 3:
// 'VCU_GEARACT_REVERSE', 4: 'VCU_GEARACT_PARK'}, 'precision': 1.0, 'len': 3,
// 'name': 'vcu_gearact', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|7]', 'bit': 15, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_vcu_1_312::Vcu_gearactType Scuvcu1312::vcu_gearact(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 3);

  Scu_vcu_1_312::Vcu_gearactType ret =
      static_cast<Scu_vcu_1_312::Vcu_gearactType>(x);
  return ret;
}
}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
