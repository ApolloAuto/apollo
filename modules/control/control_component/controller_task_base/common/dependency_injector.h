/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/control_msgs/control_interactive_msg.pb.h"
#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/control/control_component/proto/control_debug.pb.h"

#include "modules/common/vehicle_state/vehicle_state_provider.h"

namespace apollo {
namespace control {

class DependencyInjector {
 public:
  DependencyInjector() = default;

  ~DependencyInjector() = default;

  apollo::common::VehicleStateProvider* vehicle_state() {
    return &vehicle_state_;
  }

  void Set_pervious_control_command(ControlCommand* control_command) {
    ADEBUG << "Get the new control_command: "
           << control_command->ShortDebugString();
    lon_debug_ = control_command->mutable_debug()->simple_lon_debug();
  }

  const SimpleLongitudinalDebug* Get_previous_lon_debug_info() const {
    return &lon_debug_;
  }

  void set_planning_command_status(
      const external_command::CommandStatus& planning_command_status) {
    planning_command_status_.CopyFrom(planning_command_status);
    ADEBUG << "Received planning_command_status_ is "
           << planning_command_status_.DebugString();
  }

  const external_command::CommandStatus* get_planning_command_status() const {
    return &planning_command_status_;
  }

  ControlCommand* previous_control_command_mutable() {
    return &control_command_;
  }

  const ControlCommand& previous_control_command() const {
    return control_command_;
  }

  ControlDebugInfo* previous_control_debug_mutable() {
    return &control_debug_previous_;
  }

  const ControlDebugInfo& previous_control_debug() const {
    return control_debug_previous_;
  }

  ControlDebugInfo* mutable_control_debug_info() {
    return &control_debug_info_;
  }

  const ControlDebugInfo& control_debug_info() const {
    return control_debug_info_;
  }

  void control_debug_info_clear() { control_debug_info_.Clear(); }

  void set_control_process(bool control_process) {
    control_process_ = control_process;
    ADEBUG << "set control_process_ is " << control_process_;
  }

  const bool control_process() const { return control_process_; }

  ControlInteractiveMsg* mutable_control_interactive_info() {
    return &control_interactive_msg_;
  }

  const ControlInteractiveMsg& control_interactive_info() const {
    return control_interactive_msg_;
  }

 private:
  apollo::common::VehicleStateProvider vehicle_state_;
  SimpleLongitudinalDebug lon_debug_;
  external_command::CommandStatus planning_command_status_;
  ControlDebugInfo control_debug_info_;
  ControlDebugInfo control_debug_previous_;
  ControlCommand control_command_;
  ControlInteractiveMsg control_interactive_msg_;
  bool control_process_ = false;
};

}  // namespace control
}  // namespace apollo
