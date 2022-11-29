/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/lexus/protocol/dash_controls_right_rpt_210.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

const int32_t Dashcontrolsrightrpt210::ID = 0x210;

// public
Dashcontrolsrightrpt210::Dashcontrolsrightrpt210() { Reset(); }

uint32_t Dashcontrolsrightrpt210::GetPeriod() const {
  // TODO(QiL) modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Dashcontrolsrightrpt210::UpdateData(uint8_t* data) {
  set_p_output_value(data, output_value_);
  set_p_commanded_value(data, commanded_value_);
  set_p_vehicle_fault(data, vehicle_fault_);
  set_p_pacmod_fault(data, pacmod_fault_);
  set_p_override_active(data, override_active_);
  set_p_output_reported_fault(data, output_reported_fault_);
  set_p_input_output_fault(data, input_output_fault_);
  set_p_enabled(data, enabled_);
  set_p_command_output_fault(data, command_output_fault_);
  set_p_manual_input(data, manual_input_);
}

void Dashcontrolsrightrpt210::Reset() {
  // TODO(QiL) you should check this manually
  output_value_ = Dash_controls_right_rpt_210::OUTPUT_VALUE_DASH_CONTROL_NONE;
  commanded_value_ =
      Dash_controls_right_rpt_210::COMMANDED_VALUE_DASH_CONTROL_NONE;
  vehicle_fault_ = false;
  pacmod_fault_ = false;
  override_active_ = false;
  output_reported_fault_ = false;
  input_output_fault_ = false;
  enabled_ = false;
  command_output_fault_ = false;
  manual_input_ = Dash_controls_right_rpt_210::MANUAL_INPUT_DASH_CONTROL_NONE;
}

Dashcontrolsrightrpt210* Dashcontrolsrightrpt210::set_output_value(
    Dash_controls_right_rpt_210::Output_valueType output_value) {
  output_value_ = output_value;
  return this;
}

// config detail: {'name': 'OUTPUT_VALUE', 'enum': {0:
// 'OUTPUT_VALUE_DASH_CONTROL_NONE', 1: 'OUTPUT_VALUE_DASH_CONTROL_OK', 2:
// 'OUTPUT_VALUE_DASH_CONTROL_LEFT', 3: 'OUTPUT_VALUE_DASH_CONTROL_RIGHT', 4:
// 'OUTPUT_VALUE_DASH_CONTROL_UP', 5: 'OUTPUT_VALUE_DASH_CONTROL_DOWN'},
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|255]', 'bit': 31, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Dashcontrolsrightrpt210::set_p_output_value(
    uint8_t* data, Dash_controls_right_rpt_210::Output_valueType output_value) {
  int x = output_value;

  Byte to_set(data + 3);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

Dashcontrolsrightrpt210* Dashcontrolsrightrpt210::set_commanded_value(
    Dash_controls_right_rpt_210::Commanded_valueType commanded_value) {
  commanded_value_ = commanded_value;
  return this;
}

// config detail: {'name': 'COMMANDED_VALUE', 'enum': {0:
// 'COMMANDED_VALUE_DASH_CONTROL_NONE', 1: 'COMMANDED_VALUE_DASH_CONTROL_OK', 2:
// 'COMMANDED_VALUE_DASH_CONTROL_LEFT', 3: 'COMMANDED_VALUE_DASH_CONTROL_RIGHT',
// 4: 'COMMANDED_VALUE_DASH_CONTROL_UP', 5:
// 'COMMANDED_VALUE_DASH_CONTROL_DOWN'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit':
// 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Dashcontrolsrightrpt210::set_p_commanded_value(
    uint8_t* data,
    Dash_controls_right_rpt_210::Commanded_valueType commanded_value) {
  int x = commanded_value;

  Byte to_set(data + 2);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

Dashcontrolsrightrpt210* Dashcontrolsrightrpt210::set_vehicle_fault(
    bool vehicle_fault) {
  vehicle_fault_ = vehicle_fault;
  return this;
}

// config detail: {'name': 'VEHICLE_FAULT', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 6,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
void Dashcontrolsrightrpt210::set_p_vehicle_fault(uint8_t* data,
                                                  bool vehicle_fault) {
  int x = vehicle_fault;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 6, 1);
}

Dashcontrolsrightrpt210* Dashcontrolsrightrpt210::set_pacmod_fault(
    bool pacmod_fault) {
  pacmod_fault_ = pacmod_fault;
  return this;
}

// config detail: {'name': 'PACMOD_FAULT', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 5,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
void Dashcontrolsrightrpt210::set_p_pacmod_fault(uint8_t* data,
                                                 bool pacmod_fault) {
  int x = pacmod_fault;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 5, 1);
}

Dashcontrolsrightrpt210* Dashcontrolsrightrpt210::set_override_active(
    bool override_active) {
  override_active_ = override_active;
  return this;
}

// config detail: {'name': 'OVERRIDE_ACTIVE', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
void Dashcontrolsrightrpt210::set_p_override_active(uint8_t* data,
                                                    bool override_active) {
  int x = override_active;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 1, 1);
}

Dashcontrolsrightrpt210* Dashcontrolsrightrpt210::set_output_reported_fault(
    bool output_reported_fault) {
  output_reported_fault_ = output_reported_fault;
  return this;
}

// config detail: {'name': 'OUTPUT_REPORTED_FAULT', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 4, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
void Dashcontrolsrightrpt210::set_p_output_reported_fault(
    uint8_t* data, bool output_reported_fault) {
  int x = output_reported_fault;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 4, 1);
}

Dashcontrolsrightrpt210* Dashcontrolsrightrpt210::set_input_output_fault(
    bool input_output_fault) {
  input_output_fault_ = input_output_fault;
  return this;
}

// config detail: {'name': 'INPUT_OUTPUT_FAULT', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 3, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
void Dashcontrolsrightrpt210::set_p_input_output_fault(
    uint8_t* data, bool input_output_fault) {
  int x = input_output_fault;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 3, 1);
}

Dashcontrolsrightrpt210* Dashcontrolsrightrpt210::set_enabled(bool enabled) {
  enabled_ = enabled;
  return this;
}

// config detail: {'name': 'ENABLED', 'offset': 0.0, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
void Dashcontrolsrightrpt210::set_p_enabled(uint8_t* data, bool enabled) {
  int x = enabled;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 1);
}

Dashcontrolsrightrpt210* Dashcontrolsrightrpt210::set_command_output_fault(
    bool command_output_fault) {
  command_output_fault_ = command_output_fault;
  return this;
}

// config detail: {'name': 'COMMAND_OUTPUT_FAULT', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 2, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
void Dashcontrolsrightrpt210::set_p_command_output_fault(
    uint8_t* data, bool command_output_fault) {
  int x = command_output_fault;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 2, 1);
}

Dashcontrolsrightrpt210* Dashcontrolsrightrpt210::set_manual_input(
    Dash_controls_right_rpt_210::Manual_inputType manual_input) {
  manual_input_ = manual_input;
  return this;
}

// config detail: {'name': 'MANUAL_INPUT', 'enum': {0:
// 'MANUAL_INPUT_DASH_CONTROL_NONE', 1: 'MANUAL_INPUT_DASH_CONTROL_OK', 2:
// 'MANUAL_INPUT_DASH_CONTROL_LEFT', 3: 'MANUAL_INPUT_DASH_CONTROL_RIGHT', 4:
// 'MANUAL_INPUT_DASH_CONTROL_UP', 5: 'MANUAL_INPUT_DASH_CONTROL_DOWN'},
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|255]', 'bit': 15, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Dashcontrolsrightrpt210::set_p_manual_input(
    uint8_t* data, Dash_controls_right_rpt_210::Manual_inputType manual_input) {
  int x = manual_input;

  Byte to_set(data + 1);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
