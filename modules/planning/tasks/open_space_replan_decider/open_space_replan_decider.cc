/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file
 **/

#include "modules/planning/tasks/open_space_replan_decider/open_space_replan_decider.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

bool OpenSpaceReplanDecider::Init(
    const std::string &config_dir, const std::string &name,
    const std::shared_ptr<DependencyInjector> &injector) {
  if (!Decider::Init(config_dir, name, injector)) {
    return false;
  }
  return Decider::LoadConfig<OpenSpaceReplanDeciderConfig>(&config_);
}

Status OpenSpaceReplanDecider::Process(Frame *frame) {
  const auto& previous_frame = injector_->frame_history()->Latest();
  if (frame == nullptr || previous_frame == nullptr) {
    const std::string msg =
        "Invalid frame, fail to process the OpenSpaceReplanDecider.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  if (previous_frame->open_space_info().optimizer_trajectory_data().empty() &&
      injector_->vehicle_state()->linear_velocity() <
      common::VehicleConfigHelper::GetConfig().
          vehicle_param().max_abs_speed_when_stopped()) {
    frame->mutable_open_space_info()->set_replan_flag(true);
  } else {
    frame->mutable_open_space_info()->set_replan_flag(false);
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
