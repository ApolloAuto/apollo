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

#include "modules/canbus/vehicle/hunter2/protocol/status_setting_441.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

using ::apollo::drivers::canbus::Byte;

const int32_t Statussetting441::ID = 0x441;

// public
Statussetting441::Statussetting441() { Reset(); }

uint32_t Statussetting441::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Statussetting441::UpdateData(uint8_t* data) {
  set_p_error_clear_instruction(data, error_clear_instruction_);
}

void Statussetting441::Reset() {
  // TODO(All) :  you should check this manually
  error_clear_instruction_ = Status_setting_441::ERROR_CLEAR_INSTRUCTION_CLEAR_ALL_NON_CRITICAL_FAULTS;
}

Statussetting441* Statussetting441::set_error_clear_instruction(
    Status_setting_441::Error_clear_instructionType error_clear_instruction) {
  error_clear_instruction_ = error_clear_instruction;
  return this;
 }

// config detail: {'bit': 7, 'enum': {0: 'ERROR_CLEAR_INSTRUCTION_CLEAR_ALL_NON_CRITICAL_FAULTS', 1: 'ERROR_CLEAR_INSTRUCTION_CLEAR_STEER_COMMUNICATE_FAILURE', 2: 'ERROR_CLEAR_INSTRUCTION_CLEAR_BACK_RIGHT_MOTOR_FAILURE', 3: 'ERROR_CLEAR_INSTRUCTION_CLEAR_BACK_LEFT_MOTOR_FAILURE', 4: 'ERROR_CLEAR_INSTRUCTION_NON', 5: 'ERROR_CLEAR_INSTRUCTION_CLEAR_BATTERY_UNDERVOLTAGE_FAULT', 6: 'ERROR_CLEAR_INSTRUCTION_CLEAR_STEER_ENCODER_COMMUNICATE', 7: 'ERROR_CLEAR_INSTRUCTION_CELAR_REMOTE_CONTROL_SIGNAL_LOSS'}, 'is_signed_var': False, 'len': 8, 'name': 'error_clear_instruction', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Statussetting441::set_p_error_clear_instruction(uint8_t* data,
    Status_setting_441::Error_clear_instructionType error_clear_instruction) {
  int x = error_clear_instruction;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}

}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo
