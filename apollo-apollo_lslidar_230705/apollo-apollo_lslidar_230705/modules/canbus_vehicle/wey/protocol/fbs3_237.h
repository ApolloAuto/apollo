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

#include "modules/canbus_vehicle/wey/proto/wey.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace wey {

class Fbs3237 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::Wey> {
 public:
  static const int32_t ID;
  Fbs3237();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Wey* chassis) const override;

 private:
  // config detail: {'description': 'Engine speed', 'offset': 0.0,
  // 'precision': 0.125, 'len': 16, 'name': 'EngSpd', 'is_signed_var': False,
  // 'physical_range': '[0|8198.875]', 'bit': 7, 'type': 'double',
  // 'order': 'motorola', 'physical_unit': 'rpm'}
  double engspd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description':'Acceleration Pedal Position','offset':0.0,
  // 'precision': 0.3937, 'len': 8, 'name': 'AccPedalPos', 'is_signed_var':
  // False, 'physical_range': '[0|100.3935]', 'bit': 23, 'type': 'double',
  // 'order': 'motorola', 'physical_unit': '%'}
  double accpedalpos(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'EPB Switch position information', 'enum':
  // {0: 'EPBSWTICHPOSITION_NEUTRAL', 1: 'EPBSWTICHPOSITION_RELEASE',
  // 2: 'EPBSWTICHPOSITION_APPLY', 3: 'EPBSWTICHPOSITION_RESERVED1'},
  // 'precision': 1.0, 'len': 2, 'name': 'EPBSwtichPosition', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 31, 'type':'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Fbs3_237::EpbswtichpositionType epbswtichposition(const std::uint8_t* bytes,
                                                    const int32_t length) const;

  // config detail:{'description': 'To indicate which gear the DCT is in now ',
  // 'enum': {0: 'CURRENTGEAR_P', 1: 'CURRENTGEAR_R', 2: 'CURRENTGEAR_N',
  // 3: 'CURRENTGEAR_D'}, 'precision': 1.0, 'len': 2, 'name': 'CurrentGear',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]','bit':39,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Fbs3_237::CurrentgearType currentgear(const std::uint8_t* bytes,
                                        const int32_t length) const;

  // config detail: {'description': 'Driver Steering Interference Detected',
  // 'enum': {0:'EPS_STREEINGMODE_MANUAL',1:'EPS_STREEINGMODE_AUTOMATIC_AVAIL',
  // 2: 'EPS_STREEINGMODE_MANUAL_FROM_DRVNTERFERENCE',
  // 3: 'EPS_STREEINGMODE_MANUAL_FROM_EPS_FAILED_DETECTED',
  // 4: 'EPS_STREEINGMODE_TEMPORARY_INHIBITED', 5:'EPS_STREEINGMODE_RESERVED1',
  // 6: 'EPS_STREEINGMODE_RESERVED2', 7: 'EPS_STREEINGMODE_RESERVED3'},
  // 'precision': 1.0, 'len': 3, 'name': 'EPS_StreeingMode', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 34, 'type':'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Fbs3_237::Eps_streeingmodeType eps_streeingmode(const std::uint8_t* bytes,
                                                  const int32_t length) const;

  // config detail: {'description': 'Value of driver input torque',
  // 'offset': -22.78, 'precision': 0.1794, 'len': 8, 'name':
  // 'EPSDrvInputTrqValue', 'is_signed_var': False,
  // 'physical_range': '[-22.78|22.96]', 'bit': 47, 'type': 'double',
  // 'order': 'motorola', 'physical_unit': 'Nm'}
  double epsdrvinputtrqvalue(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'description': 'Value of consumed current by EPS',
  // 'offset': 0.0, 'precision': 0.5, 'len': 8, 'name': 'EPSConsumedCurrValue',
  // 'is_signed_var': False, 'physical_range': '[127|127]', 'bit': 55,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'A'}
  double epsconsumedcurrvalue(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'description': 'Applied steering mode currently',
  // 'enum': {0: 'EPSCURRMOD_NORMAL_MODE', 1: 'EPSCURRMOD_SPORT_MODE',
  // 2: 'EPSCURRMOD_COMFORT_MODE', 3: 'EPSCURRMOD_MODESELECTIONNOTPOSSIBLE',
  // 4: 'EPSCURRMOD_NO_DISPLAY', 5: 'EPSCURRMOD_CONDITIONNOTMEET',
  // 6: 'EPSCURRMOD_RESERVED1'}, 'precision': 1.0,'len': 3,'name':'EPSCurrMod',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]',
  // 'bit': 61, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Fbs3_237::EpscurrmodType epscurrmod(const std::uint8_t* bytes,
                                      const int32_t length) const;
};

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
