/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
namespace devkit {

class Steeringreport502 : public ::apollo::drivers::canbus::ProtocolData<
                              ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Steeringreport502();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 55, 'is_signed_var': False, 'len': 8, 'name':
  // 'Steer_ANGLE_SPD_Actual', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|0]', 'physical_unit': 'deg/s', 'precision': 1.0,
  // 'type': 'int'}
  int steer_angle_spd_actual(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 23, 'description': 'Steer system communication
  // fault', 'enum': {0: 'STEER_FLT2_NO_FAULT', 1:
  // 'STEER_FLT2_STEER_SYSTEM_COMUNICATION_FAULT'}, 'is_signed_var': False,
  // 'len': 8, 'name': 'Steer_FLT2', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Steering_report_502::Steer_flt2Type steer_flt2(const std::uint8_t* bytes,
                                                 const int32_t length) const;

  // config detail: {'bit': 15, 'description': 'Steer system hardware fault',
  // 'enum': {0: 'STEER_FLT1_NO_FAULT', 1:
  // 'STEER_FLT1_STEER_SYSTEM_HARDWARE_FAULT'}, 'is_signed_var': False, 'len':
  // 8, 'name': 'Steer_FLT1', 'offset': 0.0, 'order': 'motorola',
  // 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Steering_report_502::Steer_flt1Type steer_flt1(const std::uint8_t* bytes,
                                                 const int32_t length) const;

  // config detail: {'bit': 1, 'enum': {0: 'STEER_EN_STATE_MANUAL', 1:
  // 'STEER_EN_STATE_AUTO', 2: 'STEER_EN_STATE_TAKEOVER', 3:
  // 'STEER_EN_STATE_STANDBY'}, 'is_signed_var': False, 'len': 2, 'name':
  // 'Steer_EN_state', 'offset': 0.0, 'order': 'motorola', 'physical_range':
  // '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Steering_report_502::Steer_en_stateType steer_en_state(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name':
  // 'Steer_ANGLE_Actual', 'offset': -500.0, 'order': 'motorola',
  // 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0,
  // 'type': 'int'}
  int steer_angle_actual(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
