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

#pragma once

#include "modules/canbus_vehicle/lexus/proto/lexus.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace lexus {

class Dashcontrolsrightrpt210 : public ::apollo::drivers::canbus::ProtocolData<
                                    ::apollo::canbus::Lexus> {
 public:
  static const int32_t ID;

  Dashcontrolsrightrpt210();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'OUTPUT_VALUE', 'enum': {0:
  // 'OUTPUT_VALUE_DASH_CONTROL_NONE', 1: 'OUTPUT_VALUE_DASH_CONTROL_OK', 2:
  // 'OUTPUT_VALUE_DASH_CONTROL_LEFT', 3: 'OUTPUT_VALUE_DASH_CONTROL_RIGHT', 4:
  // 'OUTPUT_VALUE_DASH_CONTROL_UP', 5: 'OUTPUT_VALUE_DASH_CONTROL_DOWN'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|255]', 'bit': 31, 'type': 'enum', 'order':
  // 'motorola', 'physical_unit': ''}
  Dashcontrolsrightrpt210* set_output_value(
      Dash_controls_right_rpt_210::Output_valueType output_value);

  // config detail: {'name': 'COMMANDED_VALUE', 'enum': {0:
  // 'COMMANDED_VALUE_DASH_CONTROL_NONE', 1: 'COMMANDED_VALUE_DASH_CONTROL_OK',
  // 2: 'COMMANDED_VALUE_DASH_CONTROL_LEFT', 3:
  // 'COMMANDED_VALUE_DASH_CONTROL_RIGHT', 4: 'COMMANDED_VALUE_DASH_CONTROL_UP',
  // 5: 'COMMANDED_VALUE_DASH_CONTROL_DOWN'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit':
  // 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Dashcontrolsrightrpt210* set_commanded_value(
      Dash_controls_right_rpt_210::Commanded_valueType commanded_value);

  // config detail: {'name': 'VEHICLE_FAULT', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 6,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  Dashcontrolsrightrpt210* set_vehicle_fault(bool vehicle_fault);

  // config detail: {'name': 'PACMOD_FAULT', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 5,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  Dashcontrolsrightrpt210* set_pacmod_fault(bool pacmod_fault);

  // config detail: {'name': 'OVERRIDE_ACTIVE', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  Dashcontrolsrightrpt210* set_override_active(bool override_active);

  // config detail: {'name': 'OUTPUT_REPORTED_FAULT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 4, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  Dashcontrolsrightrpt210* set_output_reported_fault(
      bool output_reported_fault);

  // config detail: {'name': 'INPUT_OUTPUT_FAULT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 3, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  Dashcontrolsrightrpt210* set_input_output_fault(bool input_output_fault);

  // config detail: {'name': 'ENABLED', 'offset': 0.0, 'precision': 1.0, 'len':
  // 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  Dashcontrolsrightrpt210* set_enabled(bool enabled);

  // config detail: {'name': 'COMMAND_OUTPUT_FAULT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 2, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  Dashcontrolsrightrpt210* set_command_output_fault(bool command_output_fault);

  // config detail: {'name': 'MANUAL_INPUT', 'enum': {0:
  // 'MANUAL_INPUT_DASH_CONTROL_NONE', 1: 'MANUAL_INPUT_DASH_CONTROL_OK', 2:
  // 'MANUAL_INPUT_DASH_CONTROL_LEFT', 3: 'MANUAL_INPUT_DASH_CONTROL_RIGHT', 4:
  // 'MANUAL_INPUT_DASH_CONTROL_UP', 5: 'MANUAL_INPUT_DASH_CONTROL_DOWN'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|255]', 'bit': 15, 'type': 'enum', 'order':
  // 'motorola', 'physical_unit': ''}
  Dashcontrolsrightrpt210* set_manual_input(
      Dash_controls_right_rpt_210::Manual_inputType manual_input);

 private:
  // config detail: {'name': 'OUTPUT_VALUE', 'enum': {0:
  // 'OUTPUT_VALUE_DASH_CONTROL_NONE', 1: 'OUTPUT_VALUE_DASH_CONTROL_OK', 2:
  // 'OUTPUT_VALUE_DASH_CONTROL_LEFT', 3: 'OUTPUT_VALUE_DASH_CONTROL_RIGHT', 4:
  // 'OUTPUT_VALUE_DASH_CONTROL_UP', 5: 'OUTPUT_VALUE_DASH_CONTROL_DOWN'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|255]', 'bit': 31, 'type': 'enum', 'order':
  // 'motorola', 'physical_unit': ''}
  void set_p_output_value(
      uint8_t* data,
      Dash_controls_right_rpt_210::Output_valueType output_value);

  // config detail: {'name': 'COMMANDED_VALUE', 'enum': {0:
  // 'COMMANDED_VALUE_DASH_CONTROL_NONE', 1: 'COMMANDED_VALUE_DASH_CONTROL_OK',
  // 2: 'COMMANDED_VALUE_DASH_CONTROL_LEFT', 3:
  // 'COMMANDED_VALUE_DASH_CONTROL_RIGHT', 4: 'COMMANDED_VALUE_DASH_CONTROL_UP',
  // 5: 'COMMANDED_VALUE_DASH_CONTROL_DOWN'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit':
  // 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_commanded_value(
      uint8_t* data,
      Dash_controls_right_rpt_210::Commanded_valueType commanded_value);

  // config detail: {'name': 'VEHICLE_FAULT', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 6,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  void set_p_vehicle_fault(uint8_t* data, bool vehicle_fault);

  // config detail: {'name': 'PACMOD_FAULT', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 5,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  void set_p_pacmod_fault(uint8_t* data, bool pacmod_fault);

  // config detail: {'name': 'OVERRIDE_ACTIVE', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  void set_p_override_active(uint8_t* data, bool override_active);

  // config detail: {'name': 'OUTPUT_REPORTED_FAULT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 4, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  void set_p_output_reported_fault(uint8_t* data, bool output_reported_fault);

  // config detail: {'name': 'INPUT_OUTPUT_FAULT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 3, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  void set_p_input_output_fault(uint8_t* data, bool input_output_fault);

  // config detail: {'name': 'ENABLED', 'offset': 0.0, 'precision': 1.0, 'len':
  // 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  void set_p_enabled(uint8_t* data, bool enabled);

  // config detail: {'name': 'COMMAND_OUTPUT_FAULT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 2, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  void set_p_command_output_fault(uint8_t* data, bool command_output_fault);

  // config detail: {'name': 'MANUAL_INPUT', 'enum': {0:
  // 'MANUAL_INPUT_DASH_CONTROL_NONE', 1: 'MANUAL_INPUT_DASH_CONTROL_OK', 2:
  // 'MANUAL_INPUT_DASH_CONTROL_LEFT', 3: 'MANUAL_INPUT_DASH_CONTROL_RIGHT', 4:
  // 'MANUAL_INPUT_DASH_CONTROL_UP', 5: 'MANUAL_INPUT_DASH_CONTROL_DOWN'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|255]', 'bit': 15, 'type': 'enum', 'order':
  // 'motorola', 'physical_unit': ''}
  void set_p_manual_input(
      uint8_t* data,
      Dash_controls_right_rpt_210::Manual_inputType manual_input);

 private:
  Dash_controls_right_rpt_210::Output_valueType output_value_;
  Dash_controls_right_rpt_210::Commanded_valueType commanded_value_;
  bool vehicle_fault_;
  bool pacmod_fault_;
  bool override_active_;
  bool output_reported_fault_;
  bool input_output_fault_;
  bool enabled_;
  bool command_output_fault_;
  Dash_controls_right_rpt_210::Manual_inputType manual_input_;
};

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
