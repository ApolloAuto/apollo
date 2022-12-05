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

#include "modules/canbus_vehicle/zhongyun/proto/zhongyun.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

class Torquecontrola3 : public ::apollo::drivers::canbus::ProtocolData<
                            ::apollo::canbus::Zhongyun> {
 public:
  static const int32_t ID;

  Torquecontrola3();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'driven_torque', 'offset': 0.0, 'precision': 0.05,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 8,
  // 'type': 'double', 'order': 'intel', 'physical_unit': '%'}
  Torquecontrola3* set_driven_torque(double driven_torque);

  // config detail: {'name': 'Driven_Enable_control', 'enum': {0:
  // 'DRIVEN_ENABLE_CONTROL_DRIVE_MANUAL', 1:
  // 'DRIVEN_ENABLE_CONTROL_DRIVE_AUTO'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0,
  // 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Torquecontrola3* set_driven_enable_control(
      Torque_control_a3::Driven_enable_controlType driven_enable_control);

 private:
  // config detail: {'name': 'driven_torque', 'offset': 0.0, 'precision': 0.05,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 8,
  // 'type': 'double', 'order': 'intel', 'physical_unit': '%'}
  void set_p_driven_torque(uint8_t* data, double driven_torque);

  // config detail: {'name': 'Driven_Enable_control', 'enum': {0:
  // 'DRIVEN_ENABLE_CONTROL_DRIVE_MANUAL', 1:
  // 'DRIVEN_ENABLE_CONTROL_DRIVE_AUTO'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0,
  // 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_driven_enable_control(
      uint8_t* data,
      Torque_control_a3::Driven_enable_controlType driven_enable_control);

 private:
  double driven_torque_;
  Torque_control_a3::Driven_enable_controlType driven_enable_control_;
};

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
