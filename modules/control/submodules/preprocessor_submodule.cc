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
  if (!cyber::common::GetProtoFromFile(FLAGS_control_common_conf_file,
                                       &control_common_conf_)) {
    AERROR << "Unable to load control common conf file: "
           << FLAGS_control_common_conf_file;
    return false;
  }

  //   cyber::ReaderConfig local_view_reader_config;
  //   local_view_reader_config.channel_name = "/apollo/control/localview";
  //   local_view_reader_config.pending_queue_size = 1;
  //   local_view_reader_ =
  //       node_->CreateReader<LocalView>(FLAGS_control_local_view_topic,
  //       nullptr);
  //   AERROR << "create local view reader ....";
  //   local_view_reader_ =
  //       node_->CreateReader<LocalView>("/apollo/control/localview", nullptr);

  //   local_view_reader_ = node_->CreateReader<LocalView>(
  //       FLAGS_control_local_view_topic,
  //       [this](const std::shared_ptr<LocalView> &localview) {
  //         AINFO << "Received local view data: run local view callback.";
  //         std::lock_guard<std::mutex> lock(mutex_);
  //         local_view_->CopyFrom(*localview);
  //       });
  // CHECK(local_view_reader_);

  // Preprocessor writer
  preprocessor_writer_ =
      node_->CreateWriter<Preprocessor>(FLAGS_control_preprocessor_topic);

  CHECK(preprocessor_writer_ != nullptr);

  return true;
}

bool PreprocessorSubmodule::Proc(const std::shared_ptr<LocalView> &local_view) {
  AERROR << "Preprocessor started ....";
  Preprocessor control_preprocessor;
  //   {
  //     std::lock_guard<std::mutex> lock(mutex_);
  //     local_view_reader_->Observe();
  //     const auto &local_view_msg = local_view_reader_->GetLatestObserved();
  //     if (local_view_msg == nullptr) {
  //       AERROR << "local view msg is not ready!";
  //       return false;
  //     }
  //     // TODO(SHU): to avoid redundent copy
  //     // local_view_->CopyFrom(*local_view_msg);
  //     control_preprocessor.mutable_local_view()->CopyFrom(*local_view_msg);
  //     local_view_ = control_preprocessor.mutable_local_view();
  //   }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    control_preprocessor.mutable_local_view()->CopyFrom(*local_view);
  }
  local_view_ = control_preprocessor.mutable_local_view();
  Status status = ProducePreprocessorStatus(&control_preprocessor);
  AERROR_IF(!status.ok()) << "Failed to produce control preprocessor:"
                          << status.error_message();

  if (local_view_->mutable_pad_msg() != nullptr) {
    auto pad_message = local_view_->mutable_pad_msg();
    if (pad_message->action() == DrivingAction::RESET) {
      AINFO << "Control received RESET action!";
      estop_ = false;
      estop_reason_.clear();
    }
    pad_received_ = true;
    std::lock_guard<std::mutex> lock(mutex_);
    control_preprocessor.set_received_pad_msg(pad_received_);
  }
//   AERROR << control_preprocessor.ShortDebugString();

  preprocessor_writer_->Write(
      std::make_shared<Preprocessor>(control_preprocessor));
  AERROR << "Preprocessor finished.";

  return true;
}

Status PreprocessorSubmodule::ProducePreprocessorStatus(
    Preprocessor *control_preprocessor) {
  Status status = CheckInput(local_view_);

  if (!status.ok()) {
    AERROR_EVERY(100) << "Control input data failed: "
                      << status.error_message();
    control_preprocessor->mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    control_preprocessor->mutable_engage_advice()->set_reason(
        status.error_message());
    estop_ = true;
    estop_reason_ = status.error_message();
  } else {
    Status status_ts = CheckTimestamp(local_view_);

    if (!status_ts.ok()) {
      AERROR << "Input messages timeout";
      status = status_ts;
      if (local_view_->chassis().driving_mode() !=
          apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
        control_preprocessor->mutable_engage_advice()->set_advice(
            apollo::common::EngageAdvice::DISALLOW_ENGAGE);
        control_preprocessor->mutable_engage_advice()->set_reason(
            status.error_message());
      }
    } else {
      control_preprocessor->mutable_engage_advice()->set_advice(
          apollo::common::EngageAdvice::READY_TO_ENGAGE);
    }
  }

  // check estop
  estop_ = control_common_conf_.enable_persistent_estop()
               ? estop_ || local_view_->trajectory().estop().is_estop()
               : local_view_->trajectory().estop().is_estop();

  if (local_view_->trajectory().estop().is_estop()) {
    estop_ = true;
    estop_reason_ = absl::StrCat("estop from planning : ",
                                 local_view_->trajectory().estop().reason());
  }

  if (local_view_->trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    estop_ = true;
    estop_reason_ =
        absl::StrCat("estop for empty planning trajectory, planning headers: ",
                     local_view_->trajectory().header().ShortDebugString());
  }

  if (FLAGS_enable_gear_drive_negative_speed_protection) {
    const double kEpsilon = 0.001;
    auto first_trajectory_point = local_view_->trajectory().trajectory_point(0);
    if (local_view_->chassis().gear_location() == Chassis::GEAR_DRIVE &&
        first_trajectory_point.v() < -1 * kEpsilon) {
      estop_ = true;
      estop_reason_ = "estop for negative speed when gear_drive";
    }
  }

  if (!estop_) {
    // auto debug = control_preprocessor->mutable_input_debug();
    // debug->mutable_localization_header()->CopyFrom(
    //     local_view_->localization().header());
    // debug->mutable_canbus_header()->CopyFrom(local_view_->chassis().header());
    // debug->mutable_trajectory_header()->CopyFrom(
    //     local_view_->trajectory().header());

    if (local_view_->trajectory().is_replan()) {
      latest_replan_trajectory_header_.CopyFrom(
          local_view_->trajectory().header());
    }

    // if (latest_replan_trajectory_header_.has_sequence_num()) {
    //   debug->mutable_latest_replan_trajectory_header()->CopyFrom(
    //       latest_replan_trajectory_header_);
    // }
  } else {
    control_preprocessor->set_estop(estop_);
    control_preprocessor->set_estop_reason(estop_reason_);
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

Status PreprocessorSubmodule::CheckTimestamp(LocalView *local_view) {
  if (!control_common_conf_.enable_input_timestamp_check() ||
      control_common_conf_.is_control_test_mode()) {
    ADEBUG << "Skip input timestamp check by gflags.";
    return Status::OK();
  }

  double current_timestamp = Clock::NowInSeconds();
  double localization_diff =
      current_timestamp - local_view->localization().header().timestamp_sec();

  if (localization_diff > (control_common_conf_.max_localization_miss_num() *
                           control_common_conf_.localization_period())) {
    AERROR << "Localization msg lost for " << std::setprecision(6)
           << localization_diff << "s";
    monitor_logger_buffer_.ERROR("Localization msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Localization msg timeout");
  }

  double chassis_diff =
      current_timestamp - local_view->chassis().header().timestamp_sec();

  if (chassis_diff > (control_common_conf_.max_chassis_miss_num() *
                      control_common_conf_.chassis_period())) {
    AERROR << "Chassis msg lost for " << std::setprecision(6) << chassis_diff
           << "s";
    monitor_logger_buffer_.ERROR("Chassis msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Chassis msg timeout");
  }

  double trajectory_diff =
      current_timestamp - local_view->trajectory().header().timestamp_sec();

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
