/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License atis_control_test_mode
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/control/control_component/control_component.h"

#include "absl/strings/str_cat.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
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
using apollo::common::VehicleStateProvider;
using apollo::cyber::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

const double kDoubleEpsilon = 1e-6;

ControlComponent::ControlComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

bool ControlComponent::Init() {
  injector_ = std::make_shared<DependencyInjector>();
  init_time_ = Clock::Now();

  AINFO << "Control init, starting ...";

  ACHECK(
      cyber::common::GetProtoFromFile(FLAGS_pipeline_file, &control_pipeline_))
      << "Unable to load control pipeline file: " + FLAGS_pipeline_file;

  AINFO << "ControlTask pipeline config file: " << FLAGS_pipeline_file
        << " is loaded.";

  // initial controller agent when not using control submodules
  ADEBUG << "FLAGS_use_control_submodules: " << FLAGS_use_control_submodules;
  if (!FLAGS_is_control_ut_test_mode) {
    if (!FLAGS_use_control_submodules &&
        !control_task_agent_.Init(injector_, control_pipeline_).ok()) {
      // set controller
      ADEBUG << "original control";
      monitor_logger_buffer_.ERROR(
          "Control init controller failed! Stopping...");
      return false;
    }
  }

  cyber::ReaderConfig chassis_reader_config;
  chassis_reader_config.channel_name = FLAGS_chassis_topic;
  chassis_reader_config.pending_queue_size = FLAGS_chassis_pending_queue_size;

  chassis_reader_ =
      node_->CreateReader<Chassis>(chassis_reader_config, nullptr);
  ACHECK(chassis_reader_ != nullptr);

  cyber::ReaderConfig planning_reader_config;
  planning_reader_config.channel_name = FLAGS_planning_trajectory_topic;
  planning_reader_config.pending_queue_size = FLAGS_planning_pending_queue_size;

  trajectory_reader_ =
      node_->CreateReader<ADCTrajectory>(planning_reader_config, nullptr);
  ACHECK(trajectory_reader_ != nullptr);

  cyber::ReaderConfig planning_command_status_reader_config;
  planning_command_status_reader_config.channel_name =
      FLAGS_planning_command_status;
  planning_command_status_reader_config.pending_queue_size =
      FLAGS_planning_status_msg_pending_queue_size;
  planning_command_status_reader_ =
      node_->CreateReader<external_command::CommandStatus>(
          planning_command_status_reader_config, nullptr);
  ACHECK(planning_command_status_reader_ != nullptr);

  cyber::ReaderConfig localization_reader_config;
  localization_reader_config.channel_name = FLAGS_localization_topic;
  localization_reader_config.pending_queue_size =
      FLAGS_localization_pending_queue_size;

  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      localization_reader_config, nullptr);
  ACHECK(localization_reader_ != nullptr);

  cyber::ReaderConfig pad_msg_reader_config;
  pad_msg_reader_config.channel_name = FLAGS_pad_topic;
  pad_msg_reader_config.pending_queue_size = FLAGS_pad_msg_pending_queue_size;

  pad_msg_reader_ =
      node_->CreateReader<PadMessage>(pad_msg_reader_config, nullptr);
  ACHECK(pad_msg_reader_ != nullptr);

  if (!FLAGS_use_control_submodules) {
    control_cmd_writer_ =
        node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);
    ACHECK(control_cmd_writer_ != nullptr);
  } else {
    local_view_writer_ =
        node_->CreateWriter<LocalView>(FLAGS_control_local_view_topic);
    ACHECK(local_view_writer_ != nullptr);
  }
  control_interactive_writer_ = node_->CreateWriter<ControlInteractiveMsg>(
      FLAGS_control_interative_topic);
  ACHECK(control_interactive_writer_ != nullptr);

  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // should init_vehicle first, let car enter work status, then use status msg
  // trigger control

  AINFO << "Control default driving action is "
        << DrivingAction_Name((enum DrivingAction)FLAGS_action);
  pad_msg_.set_action((enum DrivingAction)FLAGS_action);

  return true;
}

void ControlComponent::OnPad(const std::shared_ptr<PadMessage> &pad) {
  std::lock_guard<std::mutex> lock(mutex_);
  pad_msg_.CopyFrom(*pad);
  ADEBUG << "Received Pad Msg:" << pad_msg_.DebugString();
  AERROR_IF(!pad_msg_.has_action()) << "pad message check failed!";
}

void ControlComponent::OnChassis(const std::shared_ptr<Chassis> &chassis) {
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_chassis_.CopyFrom(*chassis);
}

void ControlComponent::OnPlanning(
    const std::shared_ptr<ADCTrajectory> &trajectory) {
  ADEBUG << "Received chassis data: run trajectory callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_trajectory_.CopyFrom(*trajectory);
}

void ControlComponent::OnPlanningCommandStatus(
    const std::shared_ptr<external_command::CommandStatus>
        &planning_command_status) {
  ADEBUG << "Received plannning command status data: run planning command "
            "status callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  planning_command_status_.CopyFrom(*planning_command_status);
}

void ControlComponent::OnLocalization(
    const std::shared_ptr<LocalizationEstimate> &localization) {
  ADEBUG << "Received control data: run localization message callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_localization_.CopyFrom(*localization);
}

void ControlComponent::OnMonitor(
    const common::monitor::MonitorMessage &monitor_message) {
  for (const auto &item : monitor_message.item()) {
    if (item.log_level() == common::monitor::MonitorMessageItem::FATAL) {
      estop_ = true;
      return;
    }
  }
}

Status ControlComponent::ProduceControlCommand(
    ControlCommand *control_command) {
  Status status = CheckInput(&local_view_);
  // check data
  if (!status.ok()) {
    AERROR_EVERY(100) << "Control input data failed: "
                      << status.error_message();
    control_command->mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    control_command->mutable_engage_advice()->set_reason(
        status.error_message());
    estop_ = true;
    estop_reason_ = status.error_message();
  } else {
    estop_ = false;
    Status status_ts = CheckTimestamp(local_view_);
    if (!status_ts.ok()) {
      AERROR << "Input messages timeout";
      // Clear trajectory data to make control stop if no data received again
      // next cycle.
      // keep the history trajectory for control compute.
      // latest_trajectory_.Clear();
      estop_ = true;
      status = status_ts;
      if (local_view_.chassis().driving_mode() !=
          apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
        control_command->mutable_engage_advice()->set_advice(
            apollo::common::EngageAdvice::DISALLOW_ENGAGE);
        control_command->mutable_engage_advice()->set_reason(
            status.error_message());
      }
    } else {
      control_command->mutable_engage_advice()->set_advice(
          apollo::common::EngageAdvice::READY_TO_ENGAGE);
      estop_ = false;
    }
  }

  // check estop
  estop_ = FLAGS_enable_persistent_estop
               ? estop_ || local_view_.trajectory().estop().is_estop()
               : local_view_.trajectory().estop().is_estop();

  if (local_view_.trajectory().estop().is_estop()) {
    estop_ = true;
    estop_reason_ = "estop from planning : ";
    estop_reason_ += local_view_.trajectory().estop().reason();
  }

  if (local_view_.trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    estop_ = true;
    estop_reason_ = "estop for empty planning trajectory, planning headers: " +
                    local_view_.trajectory().header().ShortDebugString();
  }

  if (FLAGS_enable_gear_drive_negative_speed_protection) {
    const double kEpsilon = 0.001;
    auto first_trajectory_point = local_view_.trajectory().trajectory_point(0);
    if (local_view_.chassis().gear_location() == Chassis::GEAR_DRIVE &&
        first_trajectory_point.v() < -1 * kEpsilon) {
      estop_ = true;
      estop_reason_ = "estop for negative speed when gear_drive";
    }
  }

  if (!estop_) {
    if (local_view_.chassis().driving_mode() == Chassis::COMPLETE_MANUAL) {
      control_task_agent_.Reset();
      AINFO_EVERY(100) << "No estop. Reset Controllers in Manual Mode";
    }

    auto debug = control_command->mutable_debug()->mutable_input_debug();
    debug->mutable_localization_header()->CopyFrom(
        local_view_.localization().header());
    debug->mutable_canbus_header()->CopyFrom(local_view_.chassis().header());
    debug->mutable_trajectory_header()->CopyFrom(
        local_view_.trajectory().header());

    if (local_view_.trajectory().is_replan()) {
      latest_replan_trajectory_header_ = local_view_.trajectory().header();
    }

    if (latest_replan_trajectory_header_.has_sequence_num()) {
      debug->mutable_latest_replan_trajectory_header()->CopyFrom(
          latest_replan_trajectory_header_);
    }
  }

  if (!estop_) {
    if (!local_view_.trajectory().trajectory_point().empty()) {
      // controller agent
      Status status_compute = control_task_agent_.ComputeControlCommand(
          &local_view_.localization(), &local_view_.chassis(),
          &local_view_.trajectory(), control_command);
      ADEBUG << "status_compute is " << status_compute;

      if (!status_compute.ok()) {
        AERROR << "Control main function failed" << " with localization: "
               << local_view_.localization().ShortDebugString()
               << " with chassis: " << local_view_.chassis().ShortDebugString()
               << " with trajectory: "
               << local_view_.trajectory().ShortDebugString()
               << " with cmd: " << control_command->ShortDebugString()
               << " status:" << status_compute.error_message();
        estop_ = true;
        estop_reason_ = status_compute.error_message();
        status = status_compute;
      }
    }
  } else {
    control_task_agent_.Reset();
    AINFO_EVERY(10) << "Estop trigger, Reset Controllers in Auto Mode";
  }

  // if planning set estop, then no control process triggered
  if (estop_) {
    AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
    // set Estop command
    control_command->set_speed(0.0);
    control_command->set_throttle(0.0);
    control_command->set_brake(FLAGS_soft_estop_brake);
    control_command->set_acceleration(FLAGS_soft_estop_acceleration);
    control_command->set_gear_location(latest_chassis_.gear_location());
    control_command->set_parking_brake(latest_chassis_.parking_brake());
    previous_steering_command_ =
        injector_->previous_control_command_mutable()->steering_target();
    control_command->set_steering_target(previous_steering_command_);
  }
  // check signal
  if (local_view_.trajectory().decision().has_vehicle_signal()) {
    control_command->mutable_signal()->CopyFrom(
        local_view_.trajectory().decision().vehicle_signal());
  }
  return status;
}

bool ControlComponent::Proc() {
  AINFO << "control proc start.";
  const auto start_time = Clock::Now();

  injector_->control_debug_info_clear();

  chassis_reader_->Observe();
  const auto &chassis_msg = chassis_reader_->GetLatestObserved();
  if (chassis_msg == nullptr) {
    AERROR << "Chassis msg is not ready!";
    injector_->set_control_process(false);
    return false;
  }
  OnChassis(chassis_msg);

  trajectory_reader_->Observe();
  const auto &trajectory_msg = trajectory_reader_->GetLatestObserved();
  if (trajectory_msg == nullptr) {
    AERROR << "planning msg is not ready!";
  } else {
    // Check if new planning data received.
    if (latest_trajectory_.header().sequence_num() !=
        trajectory_msg->header().sequence_num()) {
      OnPlanning(trajectory_msg);
    }
  }

  planning_command_status_reader_->Observe();
  const auto &planning_status_msg =
      planning_command_status_reader_->GetLatestObserved();
  if (planning_status_msg != nullptr) {
    OnPlanningCommandStatus(planning_status_msg);
    ADEBUG << "Planning command status msg is \n"
           << planning_command_status_.ShortDebugString();
  }
  injector_->set_planning_command_status(planning_command_status_);

  localization_reader_->Observe();
  const auto &localization_msg = localization_reader_->GetLatestObserved();
  if (localization_msg == nullptr) {
    AERROR << "localization msg is not ready!";
    injector_->set_control_process(false);
    return false;
  }
  OnLocalization(localization_msg);

  pad_msg_reader_->Observe();
  const auto &pad_msg = pad_msg_reader_->GetLatestObserved();
  if (pad_msg != nullptr) {
    OnPad(pad_msg);
  }

  {
    // TODO(SHU): to avoid redundent copy
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.mutable_chassis()->CopyFrom(latest_chassis_);
    local_view_.mutable_trajectory()->CopyFrom(latest_trajectory_);
    local_view_.mutable_localization()->CopyFrom(latest_localization_);
    if (pad_msg != nullptr) {
      local_view_.mutable_pad_msg()->CopyFrom(pad_msg_);
    }
  }

  // use control submodules
  if (FLAGS_use_control_submodules) {
    local_view_.mutable_header()->set_lidar_timestamp(
        local_view_.trajectory().header().lidar_timestamp());
    local_view_.mutable_header()->set_camera_timestamp(
        local_view_.trajectory().header().camera_timestamp());
    local_view_.mutable_header()->set_radar_timestamp(
        local_view_.trajectory().header().radar_timestamp());
    common::util::FillHeader(FLAGS_control_local_view_topic, &local_view_);

    const auto end_time = Clock::Now();

    // measure latency
    static apollo::common::LatencyRecorder latency_recorder(
        FLAGS_control_local_view_topic);
    latency_recorder.AppendLatencyRecord(
        local_view_.trajectory().header().lidar_timestamp(), start_time,
        end_time);

    local_view_writer_->Write(local_view_);
    return true;
  }

  if (pad_msg != nullptr) {
    ADEBUG << "pad_msg: " << pad_msg_.ShortDebugString();
    ADEBUG << "pad_msg is not nullptr";
    if (pad_msg_.action() == DrivingAction::RESET) {
      AINFO << "Control received RESET action!";
      estop_ = false;
      estop_reason_.clear();
    }
    pad_received_ = true;
  }

  if (FLAGS_is_control_test_mode && FLAGS_control_test_duration > 0 &&
      (start_time - init_time_).ToSecond() > FLAGS_control_test_duration) {
    AERROR << "Control finished testing. exit";
    injector_->set_control_process(false);
    return false;
  }

  injector_->set_control_process(true);

  injector_->mutable_control_debug_info()
      ->mutable_control_component_debug()
      ->Clear();
  CheckAutoMode(&local_view_.chassis());

  ControlCommand control_command;

  Status status;
  if (local_view_.chassis().driving_mode() ==
      apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    status = ProduceControlCommand(&control_command);
    ADEBUG << "Produce control command normal.";
  } else {
    ADEBUG << "Into reset control command.";
    ResetAndProduceZeroControlCommand(&latest_chassis_, &control_command);
  }

  AERROR_IF(!status.ok()) << "Failed to produce control command:"
                          << status.error_message();

  if (pad_received_) {
    control_command.mutable_pad_msg()->CopyFrom(pad_msg_);
    pad_received_ = false;
  }

  // forward estop reason among following control frames.
  if (estop_) {
    control_command.mutable_header()->mutable_status()->set_msg(estop_reason_);
  }

  // set header
  control_command.mutable_header()->set_lidar_timestamp(
      local_view_.trajectory().header().lidar_timestamp());
  control_command.mutable_header()->set_camera_timestamp(
      local_view_.trajectory().header().camera_timestamp());
  control_command.mutable_header()->set_radar_timestamp(
      local_view_.trajectory().header().radar_timestamp());

  if (FLAGS_is_control_test_mode) {
    ADEBUG << "Skip publish control command in test mode";
    return true;
  }

  if (fabs(control_command.debug().simple_lon_debug().vehicle_pitch()) <
      kDoubleEpsilon) {
    injector_->vehicle_state()->Update(local_view_.localization(),
                                       local_view_.chassis());
    GetVehiclePitchAngle(&control_command);
  }

  const auto end_time = Clock::Now();
  const double time_diff_ms = (end_time - start_time).ToSecond() * 1e3;
  ADEBUG << "total control time spend: " << time_diff_ms << " ms.";

  control_command.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  control_command.mutable_latency_stats()->set_total_time_exceeded(
      time_diff_ms > FLAGS_control_period * 1e3);
  if (control_command.mutable_latency_stats()->total_time_exceeded()) {
    AINFO << "total control cycle time is exceeded: " << time_diff_ms << " ms.";
  }
  status.Save(control_command.mutable_header()->mutable_status());

  // measure latency
  if (local_view_.trajectory().header().has_lidar_timestamp()) {
    static apollo::common::LatencyRecorder latency_recorder(
        FLAGS_control_command_topic);
    latency_recorder.AppendLatencyRecord(
        local_view_.trajectory().header().lidar_timestamp(), start_time,
        end_time);
  }

  common::util::FillHeader(node_->Name(), &control_command);
  if (FLAGS_sim_by_record) {
    control_command.mutable_header()->set_timestamp_sec(
        latest_chassis_.header().timestamp_sec());
  }
  ADEBUG << control_command.ShortDebugString();

  control_cmd_writer_->Write(control_command);

  // save current control command
  injector_->Set_pervious_control_command(&control_command);
  injector_->previous_control_command_mutable()->CopyFrom(control_command);
  // save current control debug
  injector_->previous_control_debug_mutable()->CopyFrom(
      injector_->control_debug_info());

  PublishControlInteractiveMsg();
  const auto end_process_control_time = Clock::Now();
  const double process_control_time_diff =
      (end_process_control_time - start_time).ToSecond() * 1e3;
  if (control_command.mutable_latency_stats()->total_time_exceeded()) {
    AINFO << "control all spend time is exceeded.";
  }
  AINFO << "control proc finished, total time spend: "
        << process_control_time_diff << " ms.";
  return true;
}

Status ControlComponent::CheckInput(LocalView *local_view) {
  ADEBUG << "Received localization:"
         << local_view->localization().ShortDebugString();
  ADEBUG << "Received chassis:" << local_view->chassis().ShortDebugString();

  if (!local_view->trajectory().estop().is_estop() &&
      local_view->trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    const std::string msg =
        absl::StrCat("planning has no trajectory point. planning_seq_num:",
                     local_view->trajectory().header().sequence_num());
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, msg);
  }

  for (auto &trajectory_point :
       *local_view->mutable_trajectory()->mutable_trajectory_point()) {
    if (std::abs(trajectory_point.v()) < FLAGS_minimum_speed_resolution &&
        std::abs(trajectory_point.a()) < FLAGS_max_acceleration_when_stopped) {
      trajectory_point.set_v(0.0);
      trajectory_point.set_a(0.0);
    }
  }

  injector_->vehicle_state()->Update(local_view->localization(),
                                     local_view->chassis());

  return Status::OK();
}

Status ControlComponent::CheckTimestamp(const LocalView &local_view) {
  if (!FLAGS_enable_input_timestamp_check || FLAGS_is_control_test_mode) {
    ADEBUG << "Skip input timestamp check by gflags.";
    return Status::OK();
  }
  std::string err_msg = "";
  double current_timestamp = FLAGS_sim_by_record
                                 ? latest_chassis_.header().timestamp_sec()
                                 : Clock::NowInSeconds();
  double localization_diff =
      current_timestamp - local_view.localization().header().timestamp_sec();
  bool localization_consist_timeout = false;
  if (localization_diff >
      (FLAGS_max_localization_miss_num * FLAGS_localization_period)) {
    localization_consist_timeout = true;
    localization_timeout_count_++;
    AERROR << "Localization msg lost for " << std::to_string(localization_diff)
           << "s";
    AERROR << "current_timestamp: " << std::to_string(current_timestamp)
           << ", localization_timestamp: "
           << std::to_string(
                  local_view.localization().header().timestamp_sec());
    err_msg = err_msg + " Localization msg timeout. ";
  } else {
    localization_consist_timeout = false;
    localization_timeout_count_ = 0;
  }
  if (localization_consist_timeout &&
      (localization_timeout_count_ >= FLAGS_max_localization_miss_num)) {
    localization_timeout_count_ = FLAGS_max_localization_miss_num;
    AERROR << "write the monitor logger, localization msg lost "
           << localization_timeout_count_ << " times.";
    monitor_logger_buffer_.ERROR("Localization msg lost");
  }

  double chassis_diff =
      current_timestamp - local_view.chassis().header().timestamp_sec();
  bool chassis_consist_timeout = false;
  if (chassis_diff > (FLAGS_max_chassis_miss_num * FLAGS_chassis_period)) {
    chassis_consist_timeout = true;
    chassis_timeout_count_++;
    AERROR << "Chassis msg lost for " << std::to_string(chassis_diff) << "s";
    AERROR << "current_timestamp: " << std::to_string(current_timestamp)
           << ", chassis_timestamp: "
           << std::to_string(local_view.chassis().header().timestamp_sec());
    err_msg = err_msg + " Chassis msg timeout. ";
  } else {
    chassis_consist_timeout = false;
    chassis_timeout_count_ = 0;
  }
  if (chassis_consist_timeout &&
      (chassis_timeout_count_ >= FLAGS_max_chassis_miss_num)) {
    chassis_timeout_count_ = FLAGS_max_chassis_miss_num;
    AERROR << "write the monitor logger, chassis msg lost "
           << chassis_timeout_count_ << " times.";
    monitor_logger_buffer_.ERROR("Chassis msg lost");
  }

  double trajectory_diff =
      current_timestamp - local_view.trajectory().header().timestamp_sec();
  bool trajectory_consist_timeout = false;
  if (trajectory_diff >
      (FLAGS_max_planning_miss_num * FLAGS_trajectory_period)) {
    trajectory_consist_timeout = true;
    trajectory_timeout_count_++;
    AERROR << "Trajectory msg lost for " << std::to_string(trajectory_diff)
           << "s";
    AERROR << "current_timestamp: " << std::to_string(current_timestamp)
           << ", trajectory_timestamp: "
           << std::to_string(local_view.trajectory().header().timestamp_sec());
    err_msg = err_msg + " Trajectory msg lost. ";
  } else {
    trajectory_consist_timeout = false;
    trajectory_timeout_count_ = 0;
  }
  if (trajectory_consist_timeout &&
      (trajectory_timeout_count_ >= FLAGS_max_localization_miss_num)) {
    trajectory_timeout_count_ = FLAGS_max_localization_miss_num;
    AERROR << "write the monitor logger, trajectory msg lost "
           << trajectory_timeout_count_ << " times.";
    monitor_logger_buffer_.ERROR("Trajectory msg lost");
  }

  if (!err_msg.empty()) {
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, err_msg);
  } else {
    return Status::OK();
  }
}

void ControlComponent::ResetAndProduceZeroControlCommand(
    const canbus::Chassis *chassis, ControlCommand *control_command) {
  control_command->set_throttle(0.0);
  control_command->set_steering_target(0.0);
  control_command->set_steering_rate(0.0);
  control_command->set_speed(0.0);
  control_command->set_brake(0.0);
  control_command->set_gear_location(chassis->gear_location());
  control_command->set_parking_brake(chassis->parking_brake());
  control_task_agent_.Reset();
  latest_trajectory_.mutable_trajectory_point()->Clear();
  latest_trajectory_.mutable_path_point()->Clear();
  trajectory_reader_->ClearData();
}

void ControlComponent::GetVehiclePitchAngle(ControlCommand *control_command) {
  double vehicle_pitch = injector_->vehicle_state()->pitch() * 180 / M_PI;
  control_command->mutable_debug()
      ->mutable_simple_lon_debug()
      ->set_vehicle_pitch(vehicle_pitch + FLAGS_pitch_offset_deg);
}

void ControlComponent::CheckAutoMode(const canbus::Chassis *chassis) {
  if (!injector_->previous_control_debug_mutable()
           ->mutable_control_component_debug()
           ->is_auto() &&
      chassis->driving_mode() == apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    from_else_to_auto_ = true;
    AINFO << "From else to auto!!!";
  } else {
    from_else_to_auto_ = false;
  }
  ADEBUG << "from_else_to_auto_: " << from_else_to_auto_;
  injector_->mutable_control_debug_info()
      ->mutable_control_component_debug()
      ->set_from_else_to_auto(from_else_to_auto_);

  if (chassis->driving_mode() == apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    is_auto_ = true;
  } else {
    is_auto_ = false;
  }
  injector_->mutable_control_debug_info()
      ->mutable_control_component_debug()
      ->set_is_auto(is_auto_);
}

void ControlComponent::PublishControlInteractiveMsg() {
  auto control_interactive_msg = injector_->control_interactive_info();
  common::util::FillHeader(node_->Name(), &control_interactive_msg);
  ADEBUG << "control interactive msg is: "
         << control_interactive_msg.ShortDebugString();
  control_interactive_writer_->Write(control_interactive_msg);
}

}  // namespace control
}  // namespace apollo
