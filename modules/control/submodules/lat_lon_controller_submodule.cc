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

#include "modules/control/submodules/lat_lon_controller_submodule.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::Status;

LatLonControllerSubmodule::LatLonControllerSubmodule()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

LatLonControllerSubmodule::~LatLonControllerSubmodule() {}

std::string LatLonControllerSubmodule::Name() const {
  return FLAGS_lat_lon_controller_submodule_name;
}

bool LatLonControllerSubmodule::Init() {
  // lateral controller initialization
  CHECK(cyber::common::GetProtoFromFile(FLAGS_lateral_controller_conf_file,
                                        &lateral_controller_conf_))
      << "Unable to load lateral controller conf file: "
      << FLAGS_lateral_controller_conf_file;

  if (!lateral_controller_.Init(&lateral_controller_conf_).ok()) {
    monitor_logger_buffer_.ERROR(
        "Control init lateral controller failed! Stopping...");
    return false;
  }
  // longitudinal controller
  CHECK(cyber::common::GetProtoFromFile(FLAGS_longitudinal_controller_conf_file,
                                        &longitudinal_controller_conf_))
      << "Unable to load longitudinal controller conf file: " +
             FLAGS_longitudinal_controller_conf_file;

  if (!longitudinal_controller_.Init(&longitudinal_controller_conf_).ok()) {
    monitor_logger_buffer_.ERROR(
        "Control init longitudinal controller failed! Stopping...");
    return false;
  }

  control_core_writer_ =
      node_->CreateWriter<ControlCommand>(FLAGS_control_core_command_topic);

  CHECK(control_core_writer_ != nullptr);

  return true;
}

bool LatLonControllerSubmodule::Proc(
    const std::shared_ptr<Preprocessor>& preprocessor_status) {
  ControlCommand control_core_command;

  // skip produce control command when estop for MPC controller
  if (preprocessor_status->estop()) {
    return false;
  }

  Status status = ProduceControlCoreCommand(preprocessor_status->local_view(),
                                            &control_core_command);
  AERROR_IF(!status.ok()) << "Failed to produce control command:"
                          << status.error_message();
  common::util::FillHeader(Name(), &control_core_command);
  control_core_writer_->Write(
      std::make_shared<ControlCommand>(control_core_command));
  return status.ok();
}

Status LatLonControllerSubmodule::ProduceControlCoreCommand(
    const LocalView& local_view, ControlCommand* control_core_command) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (local_view.chassis().driving_mode() == Chassis::COMPLETE_MANUAL) {
    lateral_controller_.Reset();
    longitudinal_controller_.Reset();
    AINFO_EVERY(100) << "Reset Controllers in Manual Mode";
  }

  // fill out control command sequentially
  Status lateral_status = lateral_controller_.ComputeControlCommand(
      &local_view.localization(), &local_view.chassis(),
      &local_view.trajectory(), control_core_command);

  // return error if lateral status has error
  if (!lateral_status.ok()) {
    return lateral_status;
  }

  Status longitudinal_status = longitudinal_controller_.ComputeControlCommand(
      &local_view.localization(), &local_view.chassis(),
      &local_view.trajectory(), control_core_command);
  return longitudinal_status;
}

}  // namespace control
}  // namespace apollo
