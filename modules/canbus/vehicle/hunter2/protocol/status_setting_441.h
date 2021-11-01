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

class Statussetting441 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Statussetting441();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 7, 'enum': {0: 'ERROR_CLEAR_INSTRUCTION_CLEAR_ALL_NON_CRITICAL_FAULTS', 1: 'ERROR_CLEAR_INSTRUCTION_CLEAR_STEER_COMMUNICATE_FAILURE', 2: 'ERROR_CLEAR_INSTRUCTION_CLEAR_BACK_RIGHT_MOTOR_FAILURE', 3: 'ERROR_CLEAR_INSTRUCTION_CLEAR_BACK_LEFT_MOTOR_FAILURE', 4: 'ERROR_CLEAR_INSTRUCTION_NON', 5: 'ERROR_CLEAR_INSTRUCTION_CLEAR_BATTERY_UNDERVOLTAGE_FAULT', 6: 'ERROR_CLEAR_INSTRUCTION_CLEAR_STEER_ENCODER_COMMUNICATE', 7: 'ERROR_CLEAR_INSTRUCTION_CELAR_REMOTE_CONTROL_SIGNAL_LOSS'}, 'is_signed_var': False, 'len': 8, 'name': 'error_clear_instruction', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Statussetting441* set_error_clear_instruction(Status_setting_441::Error_clear_instructionType error_clear_instruction);

 private:

  // config detail: {'bit': 7, 'enum': {0: 'ERROR_CLEAR_INSTRUCTION_CLEAR_ALL_NON_CRITICAL_FAULTS', 1: 'ERROR_CLEAR_INSTRUCTION_CLEAR_STEER_COMMUNICATE_FAILURE', 2: 'ERROR_CLEAR_INSTRUCTION_CLEAR_BACK_RIGHT_MOTOR_FAILURE', 3: 'ERROR_CLEAR_INSTRUCTION_CLEAR_BACK_LEFT_MOTOR_FAILURE', 4: 'ERROR_CLEAR_INSTRUCTION_NON', 5: 'ERROR_CLEAR_INSTRUCTION_CLEAR_BATTERY_UNDERVOLTAGE_FAULT', 6: 'ERROR_CLEAR_INSTRUCTION_CLEAR_STEER_ENCODER_COMMUNICATE', 7: 'ERROR_CLEAR_INSTRUCTION_CELAR_REMOTE_CONTROL_SIGNAL_LOSS'}, 'is_signed_var': False, 'len': 8, 'name': 'error_clear_instruction', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_error_clear_instruction(uint8_t* data, Status_setting_441::Error_clear_instructionType error_clear_instruction);

 private:
  Status_setting_441::Error_clear_instructionType error_clear_instruction_;
};

}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo


