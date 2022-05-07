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

class Vcudrivefeedback18ff7097 : public ::apollo::drivers::canbus::ProtocolData<
                                     ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcudrivefeedback18ff7097();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
  // 'VCU_Drive_Life_Signal', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int vcu_drive_life_signal(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 16, 'name':
  // 'VCU_Drive_Vehicle_Speed', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': 'Km/h', 'precision': 1.0,
  // 'type': 'int'}
  int vcu_drive_vehicle_speed(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 16, 'name':
  // 'VCU_Drive_Real_Torque', 'offset': -5000.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': 'Nm', 'precision': 1.0, 'type':
  // 'int'}
  int vcu_drive_real_torque(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
  // 'VCU_Drive_Break_Pedal_Position', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.4,
  // 'type': 'double'}
  double vcu_drive_break_pedal_position(const std::uint8_t* bytes,
                                        const int32_t length) const;

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
  // 'VCU_Drive_Throttle_Pedal_Position', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.4,
  // 'type': 'double'}
  double vcu_drive_throttle_pedal_position(const std::uint8_t* bytes,
                                           const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name':
  // 'VCU_Drive_AutoDrive_Confirm', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int vcu_drive_autodrive_confirm(const std::uint8_t* bytes,
                                  const int32_t length) const;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
