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
 * @file preprocessor_submodule.cc
 */

#include "modules/control/submodules/preprocessor_submodule.h"

#include <string>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/latency_recorder/latency_recorder.h"
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

PreprocessorSubmodule::PreprocessorSubmodule()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

PreprocessorSubmodule::~PreprocessorSubmodule() {}

std::string PreprocessorSubmodule::Name() const {
  return FLAGS_preprocessor_submodule_name;
}

bool PreprocessorSubmodule::Init() {
  ACHECK(cyber::common::GetProtoFromFile(FLAGS_control_common_conf_file,
                                         &control_common_conf_))
      << "Unable to load control common conf file: "
      << FLAGS_control_common_conf_file;

  // Preprocessor writer
  preprocessor_writer_ =
      node_->CreateWriter<Preprocessor>(FLAGS_control_preprocessor_topic);

  ACHECK(preprocessor_writer_ != nullptr);
  return true;
}

bool PreprocessorSubmodule::Proc(const std::shared_ptr<LocalView> &local_view) {
  ADEBUG << "Preprocessor started ....";
  const auto start_time = Clock::Now();

  Preprocessor control_preprocessor;
  // handling estop
  auto *preprocessor_status =
      control_preprocessor.mutable_header()->mutable_status();

  control_preprocessor.mutable_local_view()->CopyFrom(*local_view);

  Status status = ProducePreprocessorStatus(&control_preprocessor);
  AERROR_IF(!status.ok()) << "Failed to produce control preprocessor:"
                          << status.error_message();

  if (status.ok() && !estop_) {
    preprocessor_status->set_error_code(ErrorCode::OK);
  } else {
    estop_ = true;
    preprocessor_status->set_error_code(status.code());
    preprocessor_status->set_msg(status.error_message());
  }

  if (control_preprocessor.local_view().has_pad_msg()) {
    const auto &pad_message = control_preprocessor.local_view().pad_msg();
    if (pad_message.action() == DrivingAction::RESET) {
      AINFO << "Control received RESET action!";
      estop_ = false;
      preprocessor_status->set_error_code(ErrorCode::OK);
      preprocessor_status->set_msg("");
    }
    control_preprocessor.set_received_pad_msg(true);
  }

  control_preprocessor.mutable_header()->set_lidar_timestamp(
      local_view->header().lidar_timestamp());
  control_preprocessor.mutable_header()->set_camera_timestamp(
      local_view->trajectory().header().camera_timestamp());
  control_preprocessor.mutable_header()->set_radar_timestamp(
      local_view->trajectory().header().radar_timestamp());
  common::util::FillHeader(Name(), &control_preprocessor);

  const auto end_time = Clock::Now();

  static apollo::common::LatencyRecorder latency_recorder(
      FLAGS_control_preprocessor_topic);
  latency_recorder.AppendLatencyRecord(
      control_preprocessor.header().lidar_timestamp(), start_time, end_time);

  preprocessor_writer_->Write(control_preprocessor);
  ADEBUG << "Preprocessor finished.";

  return true;
}

Status PreprocessorSubmodule::ProducePreprocessorStatus(
    Preprocessor *control_preprocessor) {
  // TODO(SJiang): rename this function since local view got changed in this
  // function.
  Status status = CheckInput(control_preprocessor->mutable_local_view());

  if (!status.ok()) {
    AERROR_EVERY(100) << "Control input data failed: "
                      << status.error_message();
    auto mutable_engage_advice = control_preprocessor->mutable_engage_advice();
    mutable_engage_advice->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    mutable_engage_advice->set_reason(status.error_message());
    // skip checking time stamp when failed input check
    return status;
  }

  Status status_ts = CheckTimestamp(control_preprocessor->local_view());

  if (!status_ts.ok()) {
    AERROR << "Input messages timeout";
    status = status_ts;
    if (control_preprocessor->local_view().chassis().driving_mode() !=
        apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
      control_preprocessor->mutable_engage_advice()->set_advice(
          apollo::common::EngageAdvice::DISALLOW_ENGAGE);
      control_preprocessor->mutable_engage_advice()->set_reason(
          status.error_message());
    }
    return status;
  }
  control_preprocessor->mutable_engage_advice()->set_advice(
      apollo::common::EngageAdvice::READY_TO_ENGAGE);

  estop_ =
      control_common_conf_.enable_persistent_estop()
          ? estop_ || control_preprocessor->local_view()
                          .trajectory()
                          .estop()
                          .is_estop()
          : control_preprocessor->local_view().trajectory().estop().is_estop();

  if (control_preprocessor->local_view().trajectory().estop().is_estop()) {
    return Status(
        ErrorCode::CONTROL_ESTOP_ERROR,
        absl::StrCat(
            "estop from planning : ",
            control_preprocessor->local_view().trajectory().estop().reason()));
  }

  if (FLAGS_enable_gear_drive_negative_speed_protection) {
    const double kEpsilon = 0.001;
    auto first_trajectory_point =
        control_preprocessor->local_view().trajectory().trajectory_point(0);
    if (control_preprocessor->local_view().chassis().gear_location() ==
            Chassis::GEAR_DRIVE &&
        first_trajectory_point.v() < -1 * kEpsilon) {
      return Status(
          ErrorCode::CONTROL_ESTOP_ERROR,
          absl::StrCat(
              "estop for empty planning trajectory, planning headers: ",
              control_preprocessor->local_view()
                  .trajectory()
                  .header()
                  .ShortDebugString()));
    }
  }

  auto input_debug = control_preprocessor->mutable_input_debug();
  input_debug->mutable_localization_header()->CopyFrom(
      control_preprocessor->local_view().localization().header());
  input_debug->mutable_canbus_header()->CopyFrom(
      control_preprocessor->local_view().chassis().header());
  input_debug->mutable_trajectory_header()->CopyFrom(
      control_preprocessor->local_view().trajectory().header());

  if (control_preprocessor->local_view().trajectory().is_replan()) {
    latest_replan_trajectory_header_.CopyFrom(
        control_preprocessor->local_view().trajectory().header());
  }

  if (latest_replan_trajectory_header_.has_sequence_num()) {
    input_debug->mutable_latest_replan_trajectory_header()->CopyFrom(
        latest_replan_trajectory_header_);
  }

  return status;
}

Status PreprocessorSubmodule::CheckInput(LocalView *local_view) {
  ADEBUG << "Received localization:"
         << local_view->localization().ShortDebugString();
  ADEBUG << "Received chassis:" << local_view->chassis().ShortDebugString();

  if (!local_view->trajectory().estop().is_estop() &&
      local_view->trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    const std::string msg =
        absl::StrCat("planning has no trajectory point. planning_seq_num: ",
                     local_view->trajectory().header().sequence_num());
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, msg);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto trajectory_point : local_view->trajectory().trajectory_point()) {
      if (std::abs(trajectory_point.v()) <
              control_common_conf_.minimum_speed_resolution() &&
          std::abs(trajectory_point.a()) <
              control_common_conf_.max_acceleration_when_stopped()) {
        trajectory_point.set_v(0.0);
        trajectory_point.set_a(0.0);
      }
    }
  }
  VehicleStateProvider::Instance()->Update(local_view->localization(),
                                           local_view->chassis());

  return Status::OK();
}

Status PreprocessorSubmodule::CheckTimestamp(const LocalView &local_view) {
  if (!control_common_conf_.enable_input_timestamp_check() ||
      control_common_conf_.is_control_test_mode()) {
    ADEBUG << "Skip input timestamp check by gflags.";
    return Status::OK();
  }

  double current_timestamp = Clock::NowInSeconds();
  double localization_diff =
      current_timestamp - local_view.localization().header().timestamp_sec();

  if (localization_diff > (control_common_conf_.max_localization_miss_num() *
                           control_common_conf_.localization_period())) {
    AERROR << "Localization msg lost for " << std::setprecision(6)
           << localization_diff << "s";
    monitor_logger_buffer_.ERROR("Localization msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Localization msg timeout");
  }

  double chassis_diff =
      current_timestamp - local_view.chassis().header().timestamp_sec();

  if (chassis_diff > (control_common_conf_.max_chassis_miss_num() *
                      control_common_conf_.chassis_period())) {
    AERROR << "Chassis msg lost for " << std::setprecision(6) << chassis_diff
           << "s";
    monitor_logger_buffer_.ERROR("Chassis msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Chassis msg timeout");
  }

  double trajectory_diff =
      current_timestamp - local_view.trajectory().header().timestamp_sec();

  if (trajectory_diff > (control_common_conf_.max_planning_miss_num() *
                         control_common_conf_.trajectory_period())) {
    AERROR << "Trajectory msg lost for " << std::setprecision(6)
           << trajectory_diff << "s";
    monitor_logger_buffer_.ERROR("Trajectory msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Trajectory msg timeout");
  }

  return Status::OK();
}

}  // namespace control
}  // namespace apollo
