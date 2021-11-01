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

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

class Controlmodesetting421 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Controlmodesetting421();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 7, 'enum': {0: 'CONTROL_MODE_SETTING_STANDBY', 1: 'CONTROL_MODE_SETTING_CAN_COMMAND_CONTROL'}, 'is_signed_var': False, 'len': 8, 'name': 'control_mode_setting', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Controlmodesetting421* set_control_mode_setting(Control_mode_setting_421::Control_mode_settingType control_mode_setting);

 private:

  // config detail: {'bit': 7, 'enum': {0: 'CONTROL_MODE_SETTING_STANDBY', 1: 'CONTROL_MODE_SETTING_CAN_COMMAND_CONTROL'}, 'is_signed_var': False, 'len': 8, 'name': 'control_mode_setting', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_control_mode_setting(uint8_t* data, Control_mode_setting_421::Control_mode_settingType control_mode_setting);

 private:
  Control_mode_setting_421::Control_mode_settingType control_mode_setting_;
};

}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo


