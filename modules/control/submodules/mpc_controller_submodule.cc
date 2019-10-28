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

#include "modules/control/submodules/mpc_controller_submodule.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::common::time::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

MPCControllerSubmodule::MPCControllerSubmodule()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

MPCControllerSubmodule::~MPCControllerSubmodule() {}

std::string MPCControllerSubmodule::Name() const {
  return FLAGS_mpc_controller_submodule_name;
}

bool MPCControllerSubmodule::Init() {
  if (!cyber::common::GetProtoFromFile(FLAGS_mpc_controller_conf_file,
                                       &mpc_controller_conf_)) {
    AERROR << "Unable to load control conf file: " +
                  FLAGS_mpc_controller_conf_file;
    return false;
  }
  // MPC controller
  if (!mpc_controller_.Init(&mpc_controller_conf_).ok()) {
    monitor_logger_buffer_.ERROR(
        "MPC Control init controller failed! Stopping...");
    return false;
  }
  return true;
}

bool MPCControllerSubmodule::Proc(
    const std::shared_ptr<control::Preprocessor>& preprocessor_status) {
  ControlCommand control_command;
  local_view_ = preprocessor_status->mutable_local_view();

  Status status = ProduceControlCommand(&control_command);
  AERROR_IF(!status.ok()) << "Failed to produce control command:"
                          << status.error_message();
  control_command_writer_->Write(
      std::make_shared<ControlCommand>(control_command));
  return true;
}

Status MPCControllerSubmodule::ProduceControlCommand(
    ControlCommand* control_command) {
  std::lock_guard<std::mutex> lock(mutex_);

  // TODO(SHU): skip produce control command if estop
  if (local_view_->mutable_chassis()->driving_mode() ==
      Chassis::COMPLETE_MANUAL) {
    mpc_controller_.Reset();
    AINFO_EVERY(100) << "Reset Controllers in Manual Mode";
  }

  Status status = mpc_controller_.ComputeControlCommand(
      local_view_->mutable_localization(), local_view_->mutable_chassis(),
      local_view_->mutable_trajectory(), control_command);

  return status;
}

}  // namespace control
}  // namespace apollo
