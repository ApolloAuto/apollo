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
namespace zhongyun {

class Steeringcontrola2 : public ::apollo::drivers::canbus::ProtocolData<
                              ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Steeringcontrola2();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'Steering_target', 'offset': -1638.35, 'precision':
  // 0.05, 'len': 16, 'is_signed_var': False, 'physical_range': '[-32|32]',
  // 'bit': 8, 'type': 'double', 'order': 'intel', 'physical_unit': 'deg'}
  Steeringcontrola2* set_steering_target(double steering_target);

  // config detail: {'name': 'Steering_Enable_control', 'enum': {0:
  // 'STEERING_ENABLE_CONTROL_STEERING_MANUALCONTROL', 1:
  // 'STEERING_ENABLE_CONTROL_STEERING_AUTOCONTROL'}, 'precision': 1.0, 'len':
  // 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit':
  // 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Steeringcontrola2* set_steering_enable_control(
      Steering_control_a2::Steering_enable_controlType steering_enable_control);

 private:
  // config detail: {'name': 'Steering_target', 'offset': -1638.35, 'precision':
  // 0.05, 'len': 16, 'is_signed_var': False, 'physical_range': '[-32|32]',
  // 'bit': 8, 'type': 'double', 'order': 'intel', 'physical_unit': 'deg'}
  void set_p_steering_target(uint8_t* data, double steering_target);

  // config detail: {'name': 'Steering_Enable_control', 'enum': {0:
  // 'STEERING_ENABLE_CONTROL_STEERING_MANUALCONTROL', 1:
  // 'STEERING_ENABLE_CONTROL_STEERING_AUTOCONTROL'}, 'precision': 1.0, 'len':
  // 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit':
  // 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_steering_enable_control(
      uint8_t* data,
      Steering_control_a2::Steering_enable_controlType steering_enable_control);

 private:
  double steering_target_;
  Steering_control_a2::Steering_enable_controlType steering_enable_control_;
};

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
