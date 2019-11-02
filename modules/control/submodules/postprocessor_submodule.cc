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
PostprocessorSubmodule::PostprocessorSubmodule() {}

PostprocessorSubmodule::~PostprocessorSubmodule() {}

std::string PostprocessorSubmodule::Name() const {
  return FLAGS_postprocessor_submodule_name;
}

bool PostprocessorSubmodule::Init() {
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
    post_processor.mutable_pad_msg()->CopyFrom(preprocessor_status->pad_msg());
  }

  // control_command.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  // control_command.mutable_latency_stats()->set_total_time_exceeded(
  //     time_diff_ms > control_conf_.control_period());
  // status.Save(control_command.mutable_header()->mutable_status());

  // forward estop reason among following control frames.
  if (preprocessor_status->estop()) {
    post_processor.mutable_header()->mutable_status()->set_msg(
        preprocessor_status->estop_reason());
  }

  // // set header
  LocalView local_view = preprocessor_status->local_view();
  post_processor.mutable_header()->set_lidar_timestamp(
      local_view.mutable_trajectory()->header().lidar_timestamp());
  post_processor.mutable_header()->set_camera_timestamp(
      local_view.mutable_trajectory()->header().camera_timestamp());
  post_processor.mutable_header()->set_radar_timestamp(
      local_view.mutable_trajectory()->header().radar_timestamp());

  // common::util::FillHeader(node_->Name(), &control_command);

  //   ADEBUG << control_command.ShortDebugString();
  //   if (control_conf_.is_control_test_mode()) {
  //     ADEBUG << "Skip publish control command in test mode";
  //     return true;
  //   }

  postprocessor_writer_->Write(
      std::make_shared<ControlCommand>(post_processor));
  return true;
}

}  // namespace control
}  // namespace apollo