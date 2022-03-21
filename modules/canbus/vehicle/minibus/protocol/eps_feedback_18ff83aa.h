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
namespace minibus {

class Epsfeedback18ff83aa : public ::apollo::drivers::canbus::ProtocolData<
                                ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Epsfeedback18ff83aa();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 56, 'enum': {0: 'EPS_VCU_STATUS_READY', 1:
  // 'EPS_VCU_STATUS_AUTO_DRIVE_MODE', 2: 'EPS_VCU_STATUS_SPEED_MODE', 3:
  // 'EPS_VCU_STATUS_NULL', 4: 'EPS_VCU_STATUS_MANUAL_MODE', 5:
  // 'EPS_VCU_STATUS_MANUAL_INTERVENTION_MODE', 6: 'EPS_VCU_STATUS_WARNING', 7:
  // 'EPS_VCU_STATUS_ERROR'}, 'is_signed_var': False, 'len': 4, 'name':
  // 'EPS_VCU_Status', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Eps_feedback_18ff83aa::Eps_vcu_statusType eps_vcu_status(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
  // 'EPS_Target_Steering_Velocity', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 10.0, 'type':
  // 'double'}
  double eps_target_steering_velocity(const std::uint8_t* bytes,
                                      const int32_t length) const;

  // config detail: {'bit': 39, 'is_signed_var': False, 'len': 16, 'name':
  // 'EPS_Real_Angle', 'offset': -870.0, 'order': 'motorola', 'physical_range':
  // '[-870|870]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
  double eps_real_angle(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 23, 'is_signed_var': False, 'len': 16, 'name':
  // 'EPS_Targit_Angle', 'offset': -870.0, 'order': 'motorola',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.1, 'type':
  // 'double'}
  double eps_targit_angle(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
  // 'EPS_Wheel_Torque', 'offset': -8.96, 'order': 'intel', 'physical_range':
  // '[0|8.96]', 'physical_unit': 'Nm', 'precision': 0.07, 'type': 'double'}
  double eps_wheel_torque(const std::uint8_t* bytes,
                          const int32_t length) const;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
