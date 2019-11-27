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

/**
 * @file postprocessor_submodule.cc
 */

#include "modules/control/submodules/postprocessor_submodule.h"

#include <string>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;

std::string PostprocessorSubmodule::Name() const {
  return FLAGS_postprocessor_submodule_name;
}

bool PostprocessorSubmodule::Init() {
  CHECK(cyber::common::GetProtoFromFile(FLAGS_control_common_conf_file,
                                        &control_common_conf_))
      << "Unable to load control common conf file: "
      << FLAGS_control_common_conf_file;

  postprocessor_writer_ =
      node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);
  CHECK(postprocessor_writer_ != nullptr);
  return true;
}

bool PostprocessorSubmodule::Proc(
    const std::shared_ptr<Preprocessor>& preprocessor_status,
    const std::shared_ptr<ControlCommand>& control_core_command) {
  ControlCommand control_command;
  if (preprocessor_status->received_pad_msg()) {
    control_command.mutable_pad_msg()->CopyFrom(
        preprocessor_status->mutable_local_view()->pad_msg());
  }

  // forward estop reason among following control frames.
  // TODO(SJiang: remove preprocessor_status)
  if (preprocessor_status->estop()) {
    AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
    control_command.mutable_header()->mutable_status()->set_msg(
        preprocessor_status->estop_reason());
    control_command.set_speed(0);
    control_command.set_throttle(0);
    control_command.set_brake(control_common_conf_.soft_estop_brake());
    control_command.set_gear_location(Chassis::GEAR_DRIVE);
  } else {
    // set control command
    control_command.set_brake(control_core_command->brake());
    control_command.set_throttle(control_core_command->throttle());
    control_command.set_steering_target(
        control_core_command->steering_target());
    control_command.set_steering_rate(control_core_command->steering_rate());
    control_command.set_gear_location(control_core_command->gear_location());
    control_command.set_acceleration(control_core_command->acceleration());
  }

  // set header
  control_command.mutable_header()->set_lidar_timestamp(
      control_core_command->header().lidar_timestamp());
  control_command.mutable_header()->set_camera_timestamp(
      control_core_command->header().camera_timestamp());
  control_command.mutable_header()->set_radar_timestamp(
      control_core_command->header().radar_timestamp());

  common::util::FillHeader(Name(), &control_command);

  postprocessor_writer_->Write(control_command);

  // TODO(SHU): add debug info; add latency time
  return true;
}

}  // namespace control
}  // namespace apollo
