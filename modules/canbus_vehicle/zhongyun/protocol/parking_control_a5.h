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

class Parkingcontrola5 : public ::apollo::drivers::canbus::ProtocolData<
                             ::apollo::canbus::Zhongyun> {
 public:
  static const int32_t ID;

  Parkingcontrola5();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'Parking_target', 'enum': {0:
  // 'PARKING_TARGET_RELEASE', 1: 'PARKING_TARGET_PARKING_TRIGGER'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  Parkingcontrola5* set_parking_target(
      Parking_control_a5::Parking_targetType parking_target);

  // config detail: {'name': 'Parking_Enable_control', 'enum': {0:
  // 'PARKING_ENABLE_CONTROL_PARKING_MANUALCONTROL', 1:
  // 'PARKING_ENABLE_CONTROL_PARKING_AUTOCONTROL'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0,
  // 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Parkingcontrola5* set_parking_enable_control(
      Parking_control_a5::Parking_enable_controlType parking_enable_control);

 private:
  // config detail: {'name': 'Parking_target', 'enum': {0:
  // 'PARKING_TARGET_RELEASE', 1: 'PARKING_TARGET_PARKING_TRIGGER'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  void set_p_parking_target(
      uint8_t* data, Parking_control_a5::Parking_targetType parking_target);

  // config detail: {'name': 'Parking_Enable_control', 'enum': {0:
  // 'PARKING_ENABLE_CONTROL_PARKING_MANUALCONTROL', 1:
  // 'PARKING_ENABLE_CONTROL_PARKING_AUTOCONTROL'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0,
  // 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_parking_enable_control(
      uint8_t* data,
      Parking_control_a5::Parking_enable_controlType parking_enable_control);

 private:
  Parking_control_a5::Parking_targetType parking_target_;
  Parking_control_a5::Parking_enable_controlType parking_enable_control_;
};

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
