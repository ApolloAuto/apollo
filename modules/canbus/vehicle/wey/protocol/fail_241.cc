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

#include "modules/canbus/vehicle/wey/protocol/fail_241.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace wey {

using ::apollo::drivers::canbus::Byte;

Fail241::Fail241() {}
const int32_t Fail241::ID = 0x241;

void Fail241::Parse(const std::uint8_t* bytes, int32_t length,
                    ChassisDetail* chassis) const {
  chassis->mutable_wey()->mutable_fail_241()->set_engfail(
      engfail(bytes, length));
  chassis->mutable_wey()->mutable_fail_241()->set_espfail(
      espfail(bytes, length));
  chassis->mutable_wey()->mutable_fail_241()->set_epbfail(
      epbfail(bytes, length));
  chassis->mutable_wey()->mutable_fail_241()->set_shiftfail(
      shiftfail(bytes, length));
  chassis->mutable_wey()->mutable_fail_241()->set_epsfail(
      epsfail(bytes, length));
}

// config detail: {'description': 'Engine Fail status', 'enum': {0:
// 'ENGFAIL_NO_FAIL', 1: 'ENGFAIL_FAIL'}, 'precision': 1.0, 'len': 1, 'name':
// 'engfail', 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
// 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Fail_241::EngfailType Fail241::engfail(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  Fail_241::EngfailType ret = static_cast<Fail_241::EngfailType>(x);
  return ret;
}

// config detail: {'description': 'ESP fault', 'enum': {0:'ESPFAIL_NO_FAILURE',
// 1: 'ESPFAIL_FAILURE'}, 'precision': 1.0, 'len': 1, 'name': 'espfail',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 14,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Fail_241::EspfailType Fail241::espfail(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  Fail_241::EspfailType ret = static_cast<Fail_241::EspfailType>(x);
  return ret;
}

// config detail: {'description': 'error indication of EPB system', 'enum': {0:
// 'EPBFAIL_UNDEFINED', 1: 'EPBFAIL_NO_ERROR', 2: 'EPBFAIL_ERROR', 3:
// 'EPBFAIL_DIAGNOSIS'}, 'precision': 1.0, 'len': 2, 'name': 'epbfail',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 35,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Fail_241::EpbfailType Fail241::epbfail(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 2);

  Fail_241::EpbfailType ret = static_cast<Fail_241::EpbfailType>(x);
  return ret;
}

// config detail: {'description': 'Driver display failure messages', 'enum': {0:
// 'SHIFTFAIL_NO_FAIL', 1: 'SHIFTFAIL_TRANSMISSION_MALFUNCTION', 2:
// 'SHIFTFAIL_TRANSMISSION_P_ENGAGEMENT_FAULT', 3:
// 'SHIFTFAIL_TRANSMISSION_P_DISENGAGEMENT_FAULT', 4: 'SHIFTFAIL_RESERVED',
// 15: 'SHIFTFAIL_TRANSMISSION_LIMIT_FUNCTION'}, 'precision': 1.0, 'len': 4,
// 'name': 'shiftfail', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|15]', 'bit': 31, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Fail_241::ShiftfailType Fail241::shiftfail(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 4);

  Fail_241::ShiftfailType ret = static_cast<Fail_241::ShiftfailType>(x);
  return ret;
}

// config detail: {'description': 'Electrical steering fail status', 'enum':
// {0: 'EPSFAIL_NO_FAULT', 1: 'EPSFAIL_FAULT'}, 'precision': 1.0, 'len': 1,
// 'name': 'epsfail', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Fail_241::EpsfailType Fail241::epsfail(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(5, 1);

  Fail_241::EpsfailType ret = static_cast<Fail_241::EpsfailType>(x);
  return ret;
}
}  // namespace wey
}  // namespace canbus
}  // namespace apollo
