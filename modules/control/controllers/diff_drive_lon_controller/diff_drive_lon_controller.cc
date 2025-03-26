// Copyright 2024 daohu527@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2024-12-30
//  Author: daohu527

#include "modules/control/controllers/diff_drive_lon_controller/diff_drive_lon_controller.h"

#include "cyber/time/time.h"

namespace apollo {
namespace control {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::cyber::Time;

DiffDriveLonController::DiffDriveLonController()
    : name_("PID-based Diff drive Controller") {
  if (FLAGS_enable_csv_debug) {
    time_t rawtime;
    char name_buffer[80];
    std::time(&rawtime);
    std::tm time_tm;
    localtime_r(&rawtime, &time_tm);
    strftime(name_buffer, 80, "/tmp/speed_log__%F_%H%M%S.csv", &time_tm);
    speed_log_file_ = fopen(name_buffer, "w");
    if (speed_log_file_ == nullptr) {
      AERROR << "Fail to open file:" << name_buffer;
      FLAGS_enable_csv_debug = false;
    }
    if (speed_log_file_ != nullptr) {
      fprintf(speed_log_file_,
              "station_reference,"
              "station_error,"
              "station_error_limited,"
              "preview_station_error,"
              "speed_reference,"
              "speed_error,"
              "speed_error_limited,"
              "preview_speed_reference,"
              "preview_speed_error,"
              "is_full_stop,"
              "\r\n");

      fflush(speed_log_file_);
    }
    AINFO << name_ << " used.";
  }
}

DiffDriveLonController::~DiffDriveLonController() { CloseLogFile(); }

void DiffDriveLonController::CloseLogFile() {
  if (FLAGS_enable_csv_debug) {
    if (speed_log_file_ != nullptr) {
      fclose(speed_log_file_);
      speed_log_file_ = nullptr;
    }
  }
}

common::Status DiffDriveLonController::Init(
    std::shared_ptr<DependencyInjector> injector) {
  if (!ControlTask::LoadConfig<LonBasedPidControllerConf>(
          &lon_based_pidcontroller_conf_)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_INIT_ERROR, "failed to load control conf");
  }

  injector_ = injector;

  station_pid_controller_.Init(lon_based_pidcontroller_conf_.station_pid_conf());

  double ts = lon_based_pidcontroller_conf_.ts();
  bool enable_leadlag =
      lon_based_pidcontroller_conf_.enable_reverse_leadlag_compensation();
  if (enable_leadlag) {
    station_leadlag_controller_.Init(
        lon_based_pidcontroller_conf_.reverse_station_leadlag_conf(), ts);
  }

  return Status::OK();
}

common::Status DiffDriveLonController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory,
    control::ControlCommand *cmd) {
  localization_ = localization;
  chassis_ = chassis;
  trajectory_message_ = trajectory;

  if (trajectory_analyzer_ == nullptr ||
      trajectory_analyzer_->seq_num() !=
          trajectory_message_->header().sequence_num()) {
    trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
  }

  auto debug = cmd->mutable_debug()->mutable_simple_lon_debug();
  debug->Clear();

  double ts = lon_based_pidcontroller_conf_.ts();
  double preview_time = lon_based_pidcontroller_conf_.preview_window() * ts;
  bool enable_leadlag =
      lon_based_pidcontroller_conf_.enable_reverse_leadlag_compensation();

  ComputeLongitudinalErrors(trajectory_analyzer_.get(), preview_time, ts,
                            debug);

  double station_error_limit =
      lon_based_pidcontroller_conf_.station_error_limit();
  double station_error_limited = 0.0;
  if (lon_based_pidcontroller_conf_.enable_speed_station_preview()) {
    station_error_limited =
        common::math::Clamp(debug->preview_station_error(),
                            -station_error_limit, station_error_limit);
  } else {
    station_error_limited = common::math::Clamp(
        debug->station_error(), -station_error_limit, station_error_limit);
  }

  if (trajectory_message_->gear() == canbus::Chassis::GEAR_REVERSE) {
    station_pid_controller_.SetPID(
        lon_based_pidcontroller_conf_.reverse_station_pid_conf());
    if (enable_leadlag) {
      station_leadlag_controller_.SetLeadlag(
          lon_based_pidcontroller_conf_.reverse_station_leadlag_conf());
    }
  }

  double speed_offset =
      station_pid_controller_.Control(station_error_limited, ts);
  if (enable_leadlag) {
    speed_offset = station_leadlag_controller_.Control(speed_offset, ts);
  }

  // Check EPB
  EPB();

  double reference_spd_cmd = debug->speed_reference() + speed_offset;

  double speed_controller_input_limit =
      lon_based_pidcontroller_conf_.speed_controller_input_limit();
  double speed_controller_input_limited =
      common::math::Clamp(reference_spd_cmd, -speed_controller_input_limit,
                          speed_controller_input_limit);

  debug->set_station_error_limited(station_error_limited);
  debug->set_speed_offset(speed_offset);
  debug->set_speed_controller_input_limited(speed_controller_input_limited);
  debug->set_speed_lookup(chassis_->speed_mps());

  cmd->set_speed(speed_controller_input_limited);

  // Switch the car gear
  Shifting(cmd);

  return Status::OK();
}

void DiffDriveLonController::ComputeLongitudinalErrors(
    const TrajectoryAnalyzer *trajectory_analyzer, const double preview_time,
    const double ts, SimpleLongitudinalDebug *debug) {
  // the decomposed vehicle motion onto Frenet frame
  // s: longitudinal accumulated distance along reference trajectory
  // s_dot: longitudinal velocity along reference trajectory
  // d: lateral distance w.r.t. reference trajectory
  // d_dot: lateral distance change rate, i.e. dd/dt
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;

  auto vehicle_state = injector_->vehicle_state();
  ACHECK(vehicle_state != nullptr);

  auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
      vehicle_state->x(), vehicle_state->y());

  trajectory_analyzer->ToTrajectoryFrame(
      vehicle_state->x(), vehicle_state->y(), vehicle_state->heading(),
      vehicle_state->linear_velocity(), matched_point, &s_matched,
      &s_dot_matched, &d_matched, &d_dot_matched);

  double current_control_time = Time::Now().ToSecond();
  double preview_control_time = current_control_time + preview_time;

  TrajectoryPoint reference_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(
          current_control_time);
  TrajectoryPoint preview_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(
          preview_control_time);

  debug->mutable_current_matched_point()->mutable_path_point()->set_x(
      matched_point.x());
  debug->mutable_current_matched_point()->mutable_path_point()->set_y(
      matched_point.y());
  debug->mutable_current_reference_point()->mutable_path_point()->set_x(
      reference_point.path_point().x());
  debug->mutable_current_reference_point()->mutable_path_point()->set_y(
      reference_point.path_point().y());
  debug->mutable_preview_reference_point()->mutable_path_point()->set_x(
      preview_point.path_point().x());
  debug->mutable_preview_reference_point()->mutable_path_point()->set_y(
      preview_point.path_point().y());

  ADEBUG << "matched point:" << matched_point.DebugString();
  ADEBUG << "reference point:" << reference_point.DebugString();
  ADEBUG << "preview point:" << preview_point.DebugString();

  double heading_error = common::math::NormalizeAngle(vehicle_state->heading() -
                                                      matched_point.theta());
  double lon_speed = vehicle_state->linear_velocity() * std::cos(heading_error);
  double lon_acceleration =
      vehicle_state->linear_acceleration() * std::cos(heading_error);
  double one_minus_kappa_lat_error = 1 - reference_point.path_point().kappa() *
                                             vehicle_state->linear_velocity() *
                                             std::sin(heading_error);

  debug->set_station_reference(reference_point.path_point().s());
  debug->set_current_station(s_matched);
  debug->set_station_error(reference_point.path_point().s() - s_matched);
  debug->set_speed_reference(reference_point.v());
  debug->set_current_speed(lon_speed);
  debug->set_speed_error(reference_point.v() - s_dot_matched);
  debug->set_acceleration_reference(reference_point.a());
  debug->set_current_acceleration(lon_acceleration);
  debug->set_acceleration_error(reference_point.a() -
                                lon_acceleration / one_minus_kappa_lat_error);
  double jerk_reference =
      (debug->acceleration_reference() - previous_acceleration_reference_) / ts;
  double lon_jerk =
      (debug->current_acceleration() - previous_acceleration_) / ts;
  debug->set_jerk_reference(jerk_reference);
  debug->set_current_jerk(lon_jerk);
  debug->set_jerk_error(jerk_reference - lon_jerk / one_minus_kappa_lat_error);
  previous_acceleration_reference_ = debug->acceleration_reference();
  previous_acceleration_ = debug->current_acceleration();

  debug->set_preview_station_error(preview_point.path_point().s() - s_matched);
  debug->set_preview_speed_error(preview_point.v() - s_dot_matched);
  debug->set_preview_speed_reference(preview_point.v());
  debug->set_preview_acceleration_reference(preview_point.a());
}

bool DiffDriveLonController::Shifting(control::ControlCommand *cmd) {
  // Switch car gear condition:
  // 1. speed is lower than a threshold
  // 2. gear in NEUTRAL
  if (std::fabs(injector_->vehicle_state()->linear_velocity()) <=
          vehicle_param_.max_abs_speed_when_stopped() ||
      chassis_->gear_location() == canbus::Chassis::GEAR_NEUTRAL) {
    cmd->set_gear_location(trajectory_message_->gear());
  } else {
    cmd->set_gear_location(chassis_->gear_location());
  }
  return true;
}

bool DiffDriveLonController::EPB() {
  // The driver presses the brake pedal, and the vehicle speed is zero for 5s.
  // EnterEPB() { debug->set_is_epb_brake(true); }
  // When the driver presses the accelerator pedal and releases the brake pedal,
  // and the vehicle detects that the engine speed exceeds the set threshold
  // (such as 5km/h), the EPB is automatically released.
  // ExitEPB() { debug->set_is_epb_brake(false); }
  return true;
}

common::Status DiffDriveLonController::Reset() {
  station_pid_controller_.Reset();
  station_leadlag_controller_.Reset();
  return Status::OK();
}

std::string DiffDriveLonController::Name() const { return name_; }

void DiffDriveLonController::Stop() { CloseLogFile(); }

}  // namespace control
}  // namespace apollo
