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
#include "modules/common/latency_recorder/latency_recorder.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::time::Clock;

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
    const std::shared_ptr<ControlCommand>& control_core_command) {
  const auto start_time = Clock::Now();
  ControlCommand control_command;
  // get all fields from control_core_command for now
  control_command = *control_core_command;

  // estop handling
  if (control_core_command->header().status().error_code() != ErrorCode::OK) {
    AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
    control_command.set_speed(0);
    control_command.set_throttle(0);
    control_command.set_brake(control_common_conf_.soft_estop_brake());
    control_command.set_gear_location(Chassis::GEAR_DRIVE);
  }

  // set header
  control_command.mutable_header()->set_lidar_timestamp(
      control_core_command->header().lidar_timestamp());
  control_command.mutable_header()->set_camera_timestamp(
      control_core_command->header().camera_timestamp());
  control_command.mutable_header()->set_radar_timestamp(
      control_core_command->header().radar_timestamp());

  common::util::FillHeader(Name(), &control_command);
  const auto end_time = Clock::Now();

  // measure latency
  static apollo::common::LatencyRecorder latency_recorder(
      FLAGS_control_command_topic);
  latency_recorder.AppendLatencyRecord(
      control_command.header().lidar_timestamp(), start_time, end_time);

  postprocessor_writer_->Write(control_command);

  return true;
}

}  // namespace control
}  // namespace apollo
