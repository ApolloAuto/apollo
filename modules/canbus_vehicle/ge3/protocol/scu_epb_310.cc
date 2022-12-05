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

#include "modules/canbus_vehicle/ge3/protocol/scu_epb_310.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::drivers::canbus::Byte;

Scuepb310::Scuepb310() {}
const int32_t Scuepb310::ID = 0x310;

void Scuepb310::Parse(const std::uint8_t* bytes, int32_t length,
                      Ge3* chassis) const {
  chassis->mutable_scu_epb_310()->set_epb_intidx(
      epb_intidx(bytes, length));
  chassis->mutable_scu_epb_310()->set_epb_drvmode(
      epb_drvmode(bytes, length));
  chassis->mutable_scu_epb_310()->set_epb_sysst(
      epb_sysst(bytes, length));
  chassis->mutable_scu_epb_310()->set_epb_faultst(
      epb_faultst(bytes, length));
}

// config detail: {'description': 'EPS interrupt index', 'enum': {0:
// 'EPB_INTIDX_NOINT', 1: 'EPB_INTIDX_OVERFLOW', 2: 'EPB_INTIDX_TIMEOUT'},
// 'precision': 1.0, 'len': 3, 'name': 'epb_intidx', 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 10, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Scu_epb_310::Epb_intidxType Scuepb310::epb_intidx(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 3);

  Scu_epb_310::Epb_intidxType ret = static_cast<Scu_epb_310::Epb_intidxType>(x);
  return ret;
}

// config detail: {'description': 'EPB drive mode', 'enum': {0:
// 'EPB_DRVMODE_INVALID', 1: 'EPB_DRVMODE_MANUAL', 2: 'EPB_DRVMODE_INTERRUPT',
// 3: 'EPB_DRVMODE_AUTO'}, 'precision': 1.0, 'len': 2, 'name': 'epb_drvmode',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 6,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Scu_epb_310::Epb_drvmodeType Scuepb310::epb_drvmode(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 2);

  Scu_epb_310::Epb_drvmodeType ret =
      static_cast<Scu_epb_310::Epb_drvmodeType>(x);
  return ret;
}

// config detail: {'description': 'EPB system status', 'enum': {0:
// 'EPB_SYSST_RELEASED', 1: 'EPB_SYSST_APPLIED', 2: 'EPB_SYSST_RELEASING', 3:
// 'EPB_SYSST_FAULT', 4: 'EPB_SYSST_APPLYING', 5: 'EPB_SYSST_DISENGAGED'},
// 'precision': 1.0, 'len': 3, 'name': 'epb_sysst', 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 2, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Scu_epb_310::Epb_sysstType Scuepb310::epb_sysst(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 3);

  Scu_epb_310::Epb_sysstType ret = static_cast<Scu_epb_310::Epb_sysstType>(x);
  return ret;
}

// config detail: {'description': 'EPB fault status', 'enum': {0:
// 'EPB_FAULTST_NORMAL', 1: 'EPB_FAULTST_FAULT'}, 'precision': 1.0, 'len': 1,
// 'name': 'epb_faultst', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Scu_epb_310::Epb_faultstType Scuepb310::epb_faultst(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  Scu_epb_310::Epb_faultstType ret =
      static_cast<Scu_epb_310::Epb_faultstType>(x);
  return ret;
}
}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
