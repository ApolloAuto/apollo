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

class Gearcontrola1 : public ::apollo::drivers::canbus::ProtocolData<
                          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Gearcontrola1();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'Gear_state_target', 'enum': {1:
  // 'GEAR_STATE_TARGET_P', 2: 'GEAR_STATE_TARGET_N', 3: 'GEAR_STATE_TARGET_D',
  // 4: 'GEAR_STATE_TARGET_R', 5: 'GEAR_STATE_TARGET_INVALID'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|5]', 'bit': 8, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  Gearcontrola1* set_gear_state_target(
      Gear_control_a1::Gear_state_targetType gear_state_target);

  // config detail: {'name': 'Gear_Enable_control', 'enum': {0:
  // 'GEAR_ENABLE_CONTROL_GEAR_MANUALCONTROL', 1:
  // 'GEAR_ENABLE_CONTROL_GEAR_AUTOCONTROL'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0,
  // 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Gearcontrola1* set_gear_enable_control(
      Gear_control_a1::Gear_enable_controlType gear_enable_control);

 private:
  // config detail: {'name': 'Gear_state_target', 'enum': {1:
  // 'GEAR_STATE_TARGET_P', 2: 'GEAR_STATE_TARGET_N', 3: 'GEAR_STATE_TARGET_D',
  // 4: 'GEAR_STATE_TARGET_R', 5: 'GEAR_STATE_TARGET_INVALID'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|5]', 'bit': 8, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  void set_p_gear_state_target(
      uint8_t* data, Gear_control_a1::Gear_state_targetType gear_state_target);

  // config detail: {'name': 'Gear_Enable_control', 'enum': {0:
  // 'GEAR_ENABLE_CONTROL_GEAR_MANUALCONTROL', 1:
  // 'GEAR_ENABLE_CONTROL_GEAR_AUTOCONTROL'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0,
  // 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_gear_enable_control(
      uint8_t* data,
      Gear_control_a1::Gear_enable_controlType gear_enable_control);

 private:
  Gear_control_a1::Gear_state_targetType gear_state_target_;
  Gear_control_a1::Gear_enable_controlType gear_enable_control_;
};

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
