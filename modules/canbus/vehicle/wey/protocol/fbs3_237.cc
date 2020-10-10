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

#include "modules/canbus/vehicle/wey/protocol/fbs3_237.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace wey {

using ::apollo::drivers::canbus::Byte;

Fbs3237::Fbs3237() {}
const int32_t Fbs3237::ID = 0x237;

void Fbs3237::Parse(const std::uint8_t* bytes, int32_t length,
                    ChassisDetail* chassis) const {
  chassis->mutable_wey()->mutable_fbs3_237()->set_engspd(engspd(bytes, length));
  chassis->mutable_wey()->mutable_fbs3_237()->set_accpedalpos(
      accpedalpos(bytes, length));
  chassis->mutable_wey()->mutable_fbs3_237()->set_epbswtichposition(
      epbswtichposition(bytes, length));
  chassis->mutable_wey()->mutable_fbs3_237()->set_currentgear(
      currentgear(bytes, length));
  chassis->mutable_wey()->mutable_fbs3_237()->set_eps_streeingmode(
      eps_streeingmode(bytes, length));
  chassis->mutable_wey()->mutable_fbs3_237()->set_epsdrvinputtrqvalue(
      epsdrvinputtrqvalue(bytes, length));
  chassis->mutable_wey()->mutable_fbs3_237()->set_epsconsumedcurrvalue(
      epsconsumedcurrvalue(bytes, length));
  chassis->mutable_wey()->mutable_fbs3_237()->set_epscurrmod(
      epscurrmod(bytes, length));
  // Added for response check
  chassis->mutable_check_response()->set_is_eps_online(
      eps_streeingmode(bytes, length) == 1);
}

// config detail: {'description': 'Engine speed', 'offset': 0.0,
// 'precision': 0.125, 'len': 16, 'name': 'engspd', 'is_signed_var': False,
// 'physical_range': '[0|8198.875]', 'bit': 7, 'type': 'double',
// 'order': 'motorola', 'physical_unit': 'rpm'}
double Fbs3237::engspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.125000;
  return ret;
}

// config detail: {'description': 'Acceleration Pedal Position', 'offset': 0.0,
// 'precision': 0.3937, 'len': 8, 'name': 'accpedalpos', 'is_signed_var':False,
// 'physical_range': '[0|100.3935]', 'bit': 23, 'type': 'double',
// 'order': 'motorola', 'physical_unit': '%'}
double Fbs3237::accpedalpos(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.393700;
  return ret;
}

// config detail: {'description': 'EPB Switch position information', 'enum':
// {0: 'EPBSWTICHPOSITION_NEUTRAL', 1: 'EPBSWTICHPOSITION_RELEASE',
// 2: 'EPBSWTICHPOSITION_APPLY', 3: 'EPBSWTICHPOSITION_RESERVED1'},
// 'precision': 1.0, 'len': 2, 'name': 'epbswtichposition', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 31, 'type':
// 'enum', 'order': 'motorola', 'physical_unit': ''}
Fbs3_237::EpbswtichpositionType Fbs3237::epbswtichposition(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(6, 2);

  Fbs3_237::EpbswtichpositionType ret =
      static_cast<Fbs3_237::EpbswtichpositionType>(x);
  return ret;
}

// config detail: {'description': 'To indicate which gear the DCT is in now ',
// 'enum': {0: 'CURRENTGEAR_P', 1: 'CURRENTGEAR_R', 2: 'CURRENTGEAR_N',
// 3: 'CURRENTGEAR_D'}, 'precision': 1.0, 'len': 2, 'name': 'currentgear',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 39,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Fbs3_237::CurrentgearType Fbs3237::currentgear(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(6, 2);

  Fbs3_237::CurrentgearType ret = static_cast<Fbs3_237::CurrentgearType>(x);
  return ret;
}

// config detail: {'description': 'Driver Steering Interference Detected',
// 'enum': {0: 'EPS_STREEINGMODE_MANUAL', 1:'EPS_STREEINGMODE_AUTOMATIC_AVAIL',
// 2: 'EPS_STREEINGMODE_MANUAL_FROM_DRVNTERFERENCE',
// 3: 'EPS_STREEINGMODE_MANUAL_FROM_EPS_FAILED_DETECTED',
// 4: 'EPS_STREEINGMODE_TEMPORARY_INHIBITED', 5: 'EPS_STREEINGMODE_RESERVED1',
// 6: 'EPS_STREEINGMODE_RESERVED2', 7: 'EPS_STREEINGMODE_RESERVED3'},
// 'precision': 1.0, 'len': 3, 'name':'eps_streeingmode','is_signed_var':False,
// 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 34, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Fbs3_237::Eps_streeingmodeType Fbs3237::eps_streeingmode(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 3);

  Fbs3_237::Eps_streeingmodeType ret =
      static_cast<Fbs3_237::Eps_streeingmodeType>(x);
  return ret;
}

// config detail: {'description': 'Value of driver input torque',
// 'offset': -22.78,'precision': 0.1794,'len': 8, 'name':'epsdrvinputtrqvalue',
// 'is_signed_var': False, 'physical_range': '[-22.78|22.96]', 'bit': 47,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'Nm'}
double Fbs3237::epsdrvinputtrqvalue(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.179400 + -22.780000;
  return ret;
}

// config detail: {'description': 'Value of consumed current by EPS',
// 'offset': 0.0, 'precision': 0.5, 'len': 8, 'name': 'epsconsumedcurrvalue',
// 'is_signed_var': False, 'physical_range': '[127|127]', 'bit': 55,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'A'}
double Fbs3237::epsconsumedcurrvalue(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.500000;
  return ret;
}

// config detail: {'description': 'Applied steering mode currently', 'enum':
// {0: 'EPSCURRMOD_NORMAL_MODE', 1: 'EPSCURRMOD_SPORT_MODE',
// 2: 'EPSCURRMOD_COMFORT_MODE', 3: 'EPSCURRMOD_MODESELECTIONNOTPOSSIBLE',
// 4: 'EPSCURRMOD_NO_DISPLAY', 5: 'EPSCURRMOD_CONDITIONNOTMEET',
// 6: 'EPSCURRMOD_RESERVED1'}, 'precision': 1.0, 'len': 3, 'name':'epscurrmod',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 61,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Fbs3_237::EpscurrmodType Fbs3237::epscurrmod(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(3, 3);

  Fbs3_237::EpscurrmodType ret = static_cast<Fbs3_237::EpscurrmodType>(x);
  return ret;
}
}  // namespace wey
}  // namespace canbus
}  // namespace apollo
