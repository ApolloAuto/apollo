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
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::Status;
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
  // initialize readers and writers
  cyber::ReaderConfig chassis_reader_config;
  chassis_reader_config.channel_name = FLAGS_chassis_topic;
  chassis_reader_config.pending_queue_size = FLAGS_chassis_pending_queue_size;

  chassis_reader_ =
      node_->CreateReader<Chassis>(chassis_reader_config, nullptr);
  CHECK(chassis_reader_ != nullptr);

  cyber::ReaderConfig planning_reader_config;
  planning_reader_config.channel_name = FLAGS_planning_trajectory_topic;
  planning_reader_config.pending_queue_size = FLAGS_planning_pending_queue_size;

  trajectory_reader_ =
      node_->CreateReader<ADCTrajectory>(planning_reader_config, nullptr);
  CHECK(trajectory_reader_ != nullptr);

  cyber::ReaderConfig localization_reader_config;
  localization_reader_config.channel_name = FLAGS_localization_topic;
  localization_reader_config.pending_queue_size =
      FLAGS_localization_pending_queue_size;

  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      localization_reader_config, nullptr);
  CHECK(localization_reader_ != nullptr);

  cyber::ReaderConfig pad_msg_reader_config;
  pad_msg_reader_config.channel_name = FLAGS_pad_topic;
  pad_msg_reader_config.pending_queue_size = FLAGS_pad_msg_pending_queue_size;

  pad_msg_reader_ =
      node_->CreateReader<PadMessage>(pad_msg_reader_config, nullptr);
  CHECK(pad_msg_reader_ != nullptr);

  control_command_writer_ =
      node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);
  CHECK(control_command_writer_ != nullptr);

  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // should init_vehicle first, let car enter work status, then use status msg
  // trigger control
  AINFO << "Control default driving action is "
        << DrivingAction_Name(mpc_controller_conf_.action());
  pad_msg_.set_action(mpc_controller_conf_.action());

  return true;
}

bool MPCControllerSubmodule::Proc() {
  // will uncomment during implementation
  // double start_timestamp = Clock::NowInSeconds();

  chassis_reader_->Observe();
  const auto &chassis_msg = chassis_reader_->GetLatestObserved();
  if (chassis_msg == nullptr) {
    AERROR << "Chassis msg is not ready!";
    return false;
  }
  OnChassis(chassis_msg);

  // TODO(Shu): implementation
  ControlCommand control_command;
  Status status = ProduceControlCommand(&control_command);
  AERROR_IF(!status.ok()) << "Failed to produce control command:"
                          << status.error_message();
  control_command_writer_->Write(
      std::make_shared<ControlCommand>(control_command));
  return true;
}

Status MPCControllerSubmodule::ProduceControlCommand(
    ControlCommand *control_command) {
  Status status = mpc_controller_.ComputeControlCommand(
      &local_view_.localization, &local_view_.chassis, &local_view_.trajectory,
      control_command);
  return status;
}

void MPCControllerSubmodule::OnChassis(
    const std::shared_ptr<Chassis> &chassis) {
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_chassis_.CopyFrom(*chassis);
}

}  // namespace control
}  // namespace apollo
