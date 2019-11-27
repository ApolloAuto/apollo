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
    const std::shared_ptr<ControlCommand>& control_command) {
  ControlCommand post_processor;
  if (preprocessor_status->received_pad_msg()) {
    post_processor.mutable_pad_msg()->CopyFrom(
        preprocessor_status->mutable_local_view()->pad_msg());
  }

  // forward estop reason among following control frames.
  if (preprocessor_status->estop()) {
    AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
    post_processor.mutable_header()->mutable_status()->set_msg(
        preprocessor_status->estop_reason());
    post_processor.set_speed(0);
    post_processor.set_throttle(0);
    post_processor.set_brake(control_common_conf_.soft_estop_brake());
    post_processor.set_gear_location(Chassis::GEAR_DRIVE);
  } else {
    // set control command
    post_processor.set_brake(control_command->brake());
    post_processor.set_throttle(control_command->throttle());
    post_processor.set_steering_target(control_command->steering_target());
    post_processor.set_steering_rate(control_command->steering_rate());
    post_processor.set_gear_location(control_command->gear_location());
    post_processor.set_acceleration(control_command->acceleration());
  }

  // set header
  LocalView local_view = preprocessor_status->local_view();
  post_processor.mutable_header()->set_lidar_timestamp(
      local_view.mutable_trajectory()->header().lidar_timestamp());
  post_processor.mutable_header()->set_camera_timestamp(
      local_view.mutable_trajectory()->header().camera_timestamp());
  post_processor.mutable_header()->set_radar_timestamp(
      local_view.mutable_trajectory()->header().radar_timestamp());

  common::util::FillHeader(Name(), &post_processor);

  postprocessor_writer_->Write(post_processor);

  // TODO(SHU): add debug info; add latency time
  return true;
}

}  // namespace control
}  // namespace apollo
