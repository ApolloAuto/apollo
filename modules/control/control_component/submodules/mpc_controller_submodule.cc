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

#include "modules/control/control_component/submodules/mpc_controller_submodule.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/latency_recorder/latency_recorder.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/control_component/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::StatusPb;
using apollo::common::VehicleStateProvider;
using apollo::cyber::Clock;
using apollo::cyber::plugin_manager::PluginManager;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

MPCControllerSubmodule::MPCControllerSubmodule()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

MPCControllerSubmodule::~MPCControllerSubmodule() {}

std::string MPCControllerSubmodule::Name() const {
  return FLAGS_mpc_controller_submodule_name;
}

bool MPCControllerSubmodule::Init() {
  injector_ = std::make_shared<DependencyInjector>();

  mpc_controller_ = PluginManager::Instance()->CreateInstance<ControlTask>(
      "apollo::control::MPCController");

  if (!mpc_controller_->Init(injector_).ok()) {
    monitor_logger_buffer_.ERROR(
        "Control init MPC controller failed! Stopping...");
    return false;
  }

  control_core_writer_ =
      node_->CreateWriter<ControlCommand>(FLAGS_control_core_command_topic);
  ACHECK(control_core_writer_ != nullptr);
  return true;
}

bool MPCControllerSubmodule::Proc(
    const std::shared_ptr<Preprocessor>& preprocessor_status) {
  const auto start_time = Clock::Now();

  ControlCommand control_core_command;
  // recording pad msg
  if (preprocessor_status->received_pad_msg()) {
    control_core_command.mutable_pad_msg()->CopyFrom(
        preprocessor_status->local_view().pad_msg());
  }
  ADEBUG << "MPC controller submodule started ....";

  // skip produce control command when estop for MPC controller
  StatusPb pre_status = preprocessor_status->header().status();
  if (pre_status.error_code() != ErrorCode::OK) {
    control_core_command.mutable_header()->mutable_status()->CopyFrom(
        pre_status);
    AERROR << "Error in preprocessor submodule.";
    return false;
  }

  Status status = ProduceControlCoreCommand(preprocessor_status->local_view(),
                                            &control_core_command);
  AERROR_IF(!status.ok()) << "Failed to produce control command:"
                          << status.error_message();

  control_core_command.mutable_header()->set_lidar_timestamp(
      preprocessor_status->header().lidar_timestamp());
  control_core_command.mutable_header()->set_camera_timestamp(
      preprocessor_status->header().camera_timestamp());
  control_core_command.mutable_header()->set_radar_timestamp(
      preprocessor_status->header().radar_timestamp());
  common::util::FillHeader(Name(), &control_core_command);

  const auto end_time = Clock::Now();

  static apollo::common::LatencyRecorder latency_recorder(
      FLAGS_control_core_command_topic);
  latency_recorder.AppendLatencyRecord(
      control_core_command.header().lidar_timestamp(), start_time, end_time);

  control_core_command.mutable_header()->mutable_status()->set_error_code(
      status.code());
  control_core_command.mutable_header()->mutable_status()->set_msg(
      status.error_message());

  control_core_writer_->Write(control_core_command);

  return status.ok();
}

Status MPCControllerSubmodule::ProduceControlCoreCommand(
    const LocalView& local_view, ControlCommand* control_core_command) {
  if (local_view.chassis().driving_mode() == Chassis::COMPLETE_MANUAL) {
    mpc_controller_->Reset();
    AINFO_EVERY(100) << "Reset Controllers in Manual Mode";
  }

  Status status = mpc_controller_->ComputeControlCommand(
      &local_view.localization(), &local_view.chassis(),
      &local_view.trajectory(), control_core_command);

  ADEBUG << "MPC controller submodule finished.";

  return status;
}

}  // namespace control
}  // namespace apollo
