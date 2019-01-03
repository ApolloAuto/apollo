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

#include "modules/canbus/vehicle/ge3/protocol/scu_bcs_1_306.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

Scubcs1306::Scubcs1306() {}
const int32_t Scubcs1306::ID = 0x306;

void Scubcs1306::Parse(const std::uint8_t* bytes, int32_t length,
                       ChassisDetail* chassis) const {
  chassis->mutable_ge3()->mutable_scu_bcs_1_306()->set_bcs_aebavailable(
      bcs_aebavailable(bytes, length));
  chassis->mutable_ge3()->mutable_scu_bcs_1_306()->set_bcs_cddavailable(
      bcs_cddavailable(bytes, length));
  chassis->mutable_ge3()->mutable_scu_bcs_1_306()->set_bcs_brkpedact(
      bcs_brkpedact(bytes, length));
  chassis->mutable_ge3()->mutable_scu_bcs_1_306()->set_bcs_intidx(
      bcs_intidx(bytes, length));
  chassis->mutable_ge3()->mutable_scu_bcs_1_306()->set_bcs_vdcfaultst(
      bcs_vdcfaultst(bytes, length));
  chassis->mutable_ge3()->mutable_scu_bcs_1_306()->set_bcs_vdcactivest(
      bcs_vdcactivest(bytes, length));
  chassis->mutable_ge3()->mutable_scu_bcs_1_306()->set_bcs_absfaultst(
      bcs_absfaultst(bytes, length));
  chassis->mutable_ge3()->mutable_scu_bcs_1_306()->set_bcs_absactivest(
      bcs_absactivest(bytes, length));
  chassis->mutable_ge3()->mutable_scu_bcs_1_306()->set_bcs_faultst(
      bcs_faultst(bytes, length));
  chassis->mutable_ge3()->mutable_scu_bcs_1_306()->set_bcs_drvmode(
      bcs_drvmode(bytes, length));
  // newcode
  chassis->mutable_check_response()->set_is_esp_online(
      bcs_drvmode(bytes, length) == 3);
}

// config detail: {'description': 'VDC active status', 'enum': {0:
// 'BCS_AEBAVAILABLE_UNAVAILABLE', 1: 'BCS_AEBAVAILABLE_AVAILABLE'},
// 'precision': 1.0, 'len': 1, 'name': 'bcs_aebavailable', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 17, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Scu_bcs_1_306::Bcs_aebavailableType Scubcs1306::bcs_aebavailable(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 1);

  Scu_bcs_1_306::Bcs_aebavailableType ret =
      static_cast<Scu_bcs_1_306::Bcs_aebavailableType>(x);
  return ret;
}

// config detail: {'description': 'VDC active status', 'enum': {0:
// 'BCS_CDDAVAILABLE_UNAVAILABLE', 1: 'BCS_CDDAVAILABLE_AVAILABLE'},
// 'precision': 1.0, 'len': 1, 'name': 'bcs_cddavailable', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Scu_bcs_1_306::Bcs_cddavailableType Scubcs1306::bcs_cddavailable(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  Scu_bcs_1_306::Bcs_cddavailableType ret =
      static_cast<Scu_bcs_1_306::Bcs_cddavailableType>(x);
  return ret;
}

// config detail: {'description': 'Actual brake pedal position', 'offset': 0.0,
// 'precision': 0.1, 'len': 10, 'name': 'bcs_brkpedact', 'is_signed_var': False,
// 'physical_range': '[0|100]', 'bit': 15, 'type': 'double', 'order':
// 'motorola', 'physical_unit': '%'}
double Scubcs1306::bcs_brkpedact(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(6, 2);
  x <<= 2;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'description': 'BCS interrupt index', 'enum': {0:
// 'BCS_INTIDX_NOINT', 1: 'BCS_INTIDX_OVERFLOW', 2: 'BCS_INTIDX_TIMEOUT', 3:
// 'BCS_INTIDX_ACCPEDINT', 4: 'BCS_INTIDX_BRKPEDINT', 5: 'BCS_INTIDX_GEARINT'},
// 'precision': 1.0, 'len': 3, 'name': 'bcs_intidx', 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 21, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Scu_bcs_1_306::Bcs_intidxType Scubcs1306::bcs_intidx(const std::uint8_t* bytes,
                                                     int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(3, 3);

  Scu_bcs_1_306::Bcs_intidxType ret =
      static_cast<Scu_bcs_1_306::Bcs_intidxType>(x);
  return ret;
}

// config detail: {'description': 'VDC fault status', 'enum': {0:
// 'BCS_VDCFAULTST_NORMAL', 1: 'BCS_VDCFAULTST_FAULT'}, 'precision': 1.0, 'len':
// 1, 'name': 'bcs_vdcfaultst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_bcs_1_306::Bcs_vdcfaultstType Scubcs1306::bcs_vdcfaultst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  Scu_bcs_1_306::Bcs_vdcfaultstType ret =
      static_cast<Scu_bcs_1_306::Bcs_vdcfaultstType>(x);
  return ret;
}

// config detail: {'description': 'VDC active status', 'enum': {0:
// 'BCS_VDCACTIVEST_INACTIVE', 1: 'BCS_VDCACTIVEST_ACTIVE'}, 'precision': 1.0,
// 'len': 1, 'name': 'bcs_vdcactivest', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_bcs_1_306::Bcs_vdcactivestType Scubcs1306::bcs_vdcactivest(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  Scu_bcs_1_306::Bcs_vdcactivestType ret =
      static_cast<Scu_bcs_1_306::Bcs_vdcactivestType>(x);
  return ret;
}

// config detail: {'description': 'ABS fault status', 'enum': {0:
// 'BCS_ABSFAULTST_NORMAL', 1: 'BCS_ABSFAULTST_FAULT'}, 'precision': 1.0, 'len':
// 1, 'name': 'bcs_absfaultst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_bcs_1_306::Bcs_absfaultstType Scubcs1306::bcs_absfaultst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  Scu_bcs_1_306::Bcs_absfaultstType ret =
      static_cast<Scu_bcs_1_306::Bcs_absfaultstType>(x);
  return ret;
}

// config detail: {'description': 'ABS active status', 'enum': {0:
// 'BCS_ABSACTIVEST_INACTIVE', 1: 'BCS_ABSACTIVEST_ACTIVE'}, 'precision': 1.0,
// 'len': 1, 'name': 'bcs_absactivest', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 4, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_bcs_1_306::Bcs_absactivestType Scubcs1306::bcs_absactivest(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 1);

  Scu_bcs_1_306::Bcs_absactivestType ret =
      static_cast<Scu_bcs_1_306::Bcs_absactivestType>(x);
  return ret;
}

// config detail: {'description': 'BCS fault status', 'enum': {0:
// 'BCS_FAULTST_NORMAL', 1: 'BCS_FAULTST_FAULT'}, 'precision': 1.0, 'len': 1,
// 'name': 'bcs_faultst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_bcs_1_306::Bcs_faultstType Scubcs1306::bcs_faultst(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  Scu_bcs_1_306::Bcs_faultstType ret =
      static_cast<Scu_bcs_1_306::Bcs_faultstType>(x);
  return ret;
}

// config detail: {'description': 'BCS drive mode', 'enum': {0:
// 'BCS_DRVMODE_INVALID', 1: 'BCS_DRVMODE_MANUAL', 2: 'BCS_DRVMODE_INTERRUPT',
// 3: 'BCS_DRVMODE_AUTO'}, 'precision': 1.0, 'len': 2, 'name': 'bcs_drvmode',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Scu_bcs_1_306::Bcs_drvmodeType Scubcs1306::bcs_drvmode(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Scu_bcs_1_306::Bcs_drvmodeType ret =
      static_cast<Scu_bcs_1_306::Bcs_drvmodeType>(x);
  return ret;
}
}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
