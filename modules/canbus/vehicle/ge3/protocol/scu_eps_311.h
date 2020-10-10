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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ge3 {

class Scueps311 : public ::apollo::drivers::canbus::ProtocolData<
                      ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Scueps311();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': 'EPS interrupt index', 'enum': {0:
  // 'EPS_INTIDX_NOINT', 1: 'EPS_INTIDX_OVERFLOW', 2: 'EPS_INTIDX_TIMEOUT', 3:
  // 'EPS_INTIDX_STEERINT'}, 'precision': 1.0, 'len': 3, 'name': 'EPS_IntIdx',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 6,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Scu_eps_311::Eps_intidxType eps_intidx(const std::uint8_t* bytes,
                                         const int32_t length) const;

  // config detail: {'description': 'Steer angle speed', 'offset': 0.0,
  // 'precision': 4.0, 'len': 8, 'name': 'EPS_SteerAngleSpd', 'is_signed_var':
  // False, 'physical_range': '[0|1016]', 'bit': 15, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'deg/s'}
  double eps_steeranglespd(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'description': 'Steer angle Left + right -', 'offset':
  // -780.0, 'precision': 0.1, 'len': 16, 'name': 'EPS_SteerAngle',
  // 'is_signed_var': False, 'physical_range': '[-780|779.9]', 'bit': 23,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
  double eps_steerangle(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'EPS fault status', 'enum': {0:
  // 'EPS_FAULTST_NORMAL', 1: 'EPS_FAULTST_FAULT'}, 'precision': 1.0, 'len': 1,
  // 'name': 'EPS_FaultSt', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Scu_eps_311::Eps_faultstType eps_faultst(const std::uint8_t* bytes,
                                           const int32_t length) const;

  // config detail: {'description': 'EPS drive mode', 'enum': {0:
  // 'EPS_DRVMODE_INVALID', 1: 'EPS_DRVMODE_MANUAL', 2: 'EPS_DRVMODE_INTERRUPT',
  // 3: 'EPS_DRVMODE_AUTO'}, 'precision': 1.0, 'len': 2, 'name': 'EPS_DrvMode',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Scu_eps_311::Eps_drvmodeType eps_drvmode(const std::uint8_t* bytes,
                                           const int32_t length) const;
};

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
