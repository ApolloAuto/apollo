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

#include "modules/control/controllers/diff_drive_lat_controller/diff_drive_lat_controller.h"

namespace apollo {
namespace control {

using apollo::common::Status;
using apollo::common::TrajectoryPoint;

namespace {

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  std::tm time_tm;
  localtime_r(&raw_time, &time_tm);
  strftime(name_buffer, 80, "/tmp/steer_log_simple_optimal_%F_%H%M%S.csv",
           &time_tm);
  return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream) {
  file_stream << "current_lateral_error," << "current_ref_heading,"
              << "current_heading," << "current_heading_error,"
              << "heading_error_rate," << "lateral_error_rate,"
              << "current_curvature," << "steer_angle,"
              << "steer_angle_feedforward,"
              << "steer_angle_lateral_contribution,"
              << "steer_angle_lateral_rate_contribution,"
              << "steer_angle_heading_contribution,"
              << "steer_angle_heading_rate_contribution,"
              << "steer_angle_feedback," << "steering_position," << "v"
              << std::endl;
}

}  // namespace

DiffDriveLatController::DiffDriveLatController()
    : name_("PID-based Diff drive lat Controller") {
  if (FLAGS_enable_csv_debug) {
    steer_log_file_.open(GetLogFileName());
    steer_log_file_ << std::fixed;
    steer_log_file_ << std::setprecision(6);
    WriteHeaders(steer_log_file_);
  }
  AINFO << "Using " << name_;
}

DiffDriveLatController::~DiffDriveLatController() { CloseLogFile(); }

void DiffDriveLatController::CloseLogFile() {
  if (FLAGS_enable_csv_debug) {
    if (steer_log_file_ != nullptr) {
      fclose(steer_log_file_);
      steer_log_file_ = nullptr;
    }
  }
}

void DiffDriveLatController::InitializeFilters() {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpfCoefficients(ts_, diff_drive_lat_controller_conf_.cutoff_freq(),
                          &den, &num);
  digital_filter_.set_coefficients(den, num);
  lateral_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      diff_drive_lat_controller_conf_.mean_filter_window_size()));
  heading_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      diff_drive_lat_controller_conf_.mean_filter_window_size()));
}

bool DiffDriveLatController::LoadControlConf() {
  vehicle_param_ =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();

  ts_ = diff_drive_lat_controller_conf_.ts();
  if (ts_ <= 0.0) {
    AERROR << "[LatController] Invalid control update interval.";
    return false;
  }

  wheelbase_ = vehicle_param_.wheel_base();
  steer_ratio_ = vehicle_param_.steer_ratio();
  low_speed_bound_ = diff_drive_lat_controller_conf_.switch_speed();
  low_speed_window_ = diff_drive_lat_controller_conf_.switch_speed_window();

  const double mass_fl = diff_drive_lat_controller_conf_.mass_fl();
  const double mass_fr = diff_drive_lat_controller_conf_.mass_fr();
  const double mass_rl = diff_drive_lat_controller_conf_.mass_rl();
  const double mass_rr = diff_drive_lat_controller_conf_.mass_rr();
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);

  // moment of inertia
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  query_relative_time_ = diff_drive_lat_controller_conf_.query_relative_time();

  return true;
}

Status DiffDriveLatController::Init(
    std::shared_ptr<DependencyInjector> injector) {
  if (!ControlTask::LoadConfig<DiffDriveLatControllerConf>(
          &diff_drive_lat_controller_conf_)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_INIT_ERROR,
                  "failed to load lat control_conf");
  }

  injector_ = injector;

  if (!LoadControlConf()) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }

  InitializeFilters();

  return Status::OK();
}

void DiffDriveLatController::UpdateDrivingOrientation() {
  auto vehicle_state = injector_->vehicle_state();
  driving_orientation_ = vehicle_state->heading();

  // Reverse the driving direction if the vehicle is in reverse mode
  if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
    driving_orientation_ =
        common::math::NormalizeAngle(driving_orientation_ + M_PI);
  }
}

Status DiffDriveLatController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory,
    control::ControlCommand *cmd) {
  trajectory_message_ = trajectory;
  auto vehicle_state = injector_->vehicle_state();
  auto previous_lon_debug = injector_->Get_previous_lon_debug_info();

  if (trajectory_analyzer_ == nullptr ||
      trajectory_analyzer_->seq_num() !=
          trajectory_message_->header().sequence_num()) {
    trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
  }

  // Transform the coordinate of the planning trajectory from the center of the
  // rear-axis to the center of mass, if conditions matched
  trajectory_analyzer_.TrajectoryTransformToCOM(lr_);

  UpdateDrivingOrientation();

  SimpleLateralDebug *debug = cmd->mutable_debug()->mutable_simple_lat_debug();
  debug->Clear();

  // 1. Calculate lateral error
  const auto &com = vehicle_state->ComputeCOMPosition(lr_);
  ComputeLateralErrors(
      com.x(), com.y(), driving_orientation_, vehicle_state->linear_velocity(),
      vehicle_state->angular_velocity(), vehicle_state->linear_acceleration(),
      trajectory_analyzer_, debug, chassis);

  // 2. PID control to obtain curvature
  double curv = curv_pid_controller_.Control(debug->lateral_error(), ts);

  // 3. Convert curvature to angular velocity, rad/s
  double ang_vel = curv * vehicle_state->linear_velocity();

  // 4. Check if the steer is locked and hence the previous steer angle should
  // be executed
  if (injector_->vehicle_state()->gear() != canbus::Chassis::GEAR_REVERSE) {
    if ((std::abs(vehicle_state->linear_velocity()) <
             diff_drive_lat_controller_conf_.lock_steer_speed() ||
         previous_lon_debug->path_remain() <= 0) &&
        vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE &&
        chassis->driving_mode() == canbus::Chassis::COMPLETE_AUTO_DRIVE) {
      ADEBUG << "Into lock steer, path_remain is "
             << previous_lon_debug->path_remain() << "linear_velocity is "
             << vehicle_state->linear_velocity();
      ang_vel = pre_ang_vel_;
    }
  }

  // 5. Limit the steering command with the designed digital filter
  ang_vel = digital_filter_.Filter(ang_vel);
  ang_vel = common::math::Clamp(ang_vel, -3.0, 3.0);

  // TODO(zeor): Temporarily use steering_rate instead of angular velocity
  cmd->set_steering_rate(ang_vel);

  pre_ang_vel_ = cmd->steering_rate();

  return Status::OK();
}

void DiffDriveLatController::ComputeLateralErrors(
    const double x, const double y, const double theta, const double linear_v,
    const double angular_v, const double linear_a,
    const TrajectoryAnalyzer &trajectory_analyzer, SimpleLateralDebug *debug,
    const canbus::Chassis *chassis) {
  TrajectoryPoint target_point;

  if (diff_drive_lat_controller_conf_.query_time_nearest_point_only()) {
    target_point = trajectory_analyzer.QueryNearestPointByAbsoluteTime(
        Clock::NowInSeconds() + query_relative_time_);
  } else {
    target_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);
  }
  const double dx = x - target_point.path_point().x();
  const double dy = y - target_point.path_point().y();

  debug->mutable_current_target_point()->mutable_path_point()->set_x(
      target_point.path_point().x());
  debug->mutable_current_target_point()->mutable_path_point()->set_y(
      target_point.path_point().y());

  ADEBUG << "x point: " << x << " y point: " << y;
  ADEBUG << "match point information : " << target_point.ShortDebugString();

  const double cos_target_heading = std::cos(target_point.path_point().theta());
  const double sin_target_heading = std::sin(target_point.path_point().theta());

  double lateral_error = cos_target_heading * dy - sin_target_heading * dx;

  debug->set_lateral_error(lateral_error);

  debug->set_ref_heading(target_point.path_point().theta());
  double heading_error =
      common::math::NormalizeAngle(theta - debug->ref_heading());
  debug->set_heading_error(heading_error);

  // Within the low-high speed transition window, linerly interplolate the
  // lookahead/lookback station for "soft" prediction window switch
  double lookahead_station = 0.0;
  double lookback_station = 0.0;
  if (std::fabs(linear_v) >= low_speed_bound_) {
    lookahead_station = lookahead_station_high_speed_;
    lookback_station = lookback_station_high_speed_;
  } else if (std::fabs(linear_v) < low_speed_bound_ - low_speed_window_) {
    lookahead_station = lookahead_station_low_speed_;
    lookback_station = lookback_station_low_speed_;
  } else {
    lookahead_station = common::math::lerp(
        lookahead_station_low_speed_, low_speed_bound_ - low_speed_window_,
        lookahead_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
    lookback_station = common::math::lerp(
        lookback_station_low_speed_, low_speed_bound_ - low_speed_window_,
        lookback_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
  }

  // Estimate the heading error with look-ahead/look-back windows as feedback
  // signal for special driving scenarios
  double heading_error_feedback;
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    heading_error_feedback = heading_error;
  } else {
    auto lookahead_point = trajectory_analyzer.QueryNearestPointByRelativeTime(
        target_point.relative_time() +
        lookahead_station /
            (std::max(std::fabs(linear_v), 0.1) * std::cos(heading_error)));
    heading_error_feedback = common::math::NormalizeAngle(
        heading_error + target_point.path_point().theta() -
        lookahead_point.path_point().theta());
  }
  debug->set_heading_error_feedback(heading_error_feedback);

  // Estimate the lateral error with look-ahead/look-back windows as feedback
  // signal for special driving scenarios
  double lateral_error_feedback;
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    lateral_error_feedback =
        lateral_error - lookback_station * std::sin(heading_error);
  } else {
    lateral_error_feedback =
        lateral_error + lookahead_station * std::sin(heading_error);
  }
  debug->set_lateral_error_feedback(lateral_error_feedback);

  auto lateral_error_dot = linear_v * std::sin(heading_error);
  auto lateral_error_dot_dot = linear_a * std::sin(heading_error);
  if (FLAGS_reverse_heading_control) {
    if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
      lateral_error_dot = -lateral_error_dot;
      lateral_error_dot_dot = -lateral_error_dot_dot;
    }
  }
  auto centripetal_acceleration =
      linear_v * linear_v / wheelbase_ *
      std::tan(chassis->steering_percentage() / 100 *
               vehicle_param_.max_steer_angle() / steer_ratio_);
  debug->set_lateral_error_rate(lateral_error_dot);
  debug->set_lateral_acceleration(lateral_error_dot_dot);
  debug->set_lateral_centripetal_acceleration(centripetal_acceleration);
  debug->set_lateral_jerk(
      (debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
  previous_lateral_acceleration_ = debug->lateral_acceleration();

  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    debug->set_heading_rate(-angular_v);
  } else {
    debug->set_heading_rate(angular_v);
  }
  debug->set_ref_heading_rate(target_point.path_point().kappa() *
                              target_point.v());
  debug->set_heading_error_rate(debug->heading_rate() -
                                debug->ref_heading_rate());

  debug->set_heading_acceleration(
      (debug->heading_rate() - previous_heading_rate_) / ts_);
  debug->set_ref_heading_acceleration(
      (debug->ref_heading_rate() - previous_ref_heading_rate_) / ts_);
  debug->set_heading_error_acceleration(debug->heading_acceleration() -
                                        debug->ref_heading_acceleration());
  previous_heading_rate_ = debug->heading_rate();
  previous_ref_heading_rate_ = debug->ref_heading_rate();

  debug->set_heading_jerk(
      (debug->heading_acceleration() - previous_heading_acceleration_) / ts_);
  debug->set_ref_heading_jerk(
      (debug->ref_heading_acceleration() - previous_ref_heading_acceleration_) /
      ts_);
  debug->set_heading_error_jerk(debug->heading_jerk() -
                                debug->ref_heading_jerk());
  previous_heading_acceleration_ = debug->heading_acceleration();
  previous_ref_heading_acceleration_ = debug->ref_heading_acceleration();

  debug->set_curvature(target_point.path_point().kappa());
}

Status DiffDriveLatController::Reset() {
  curv_pid_controller_.Reset();
  return Status::OK();
}

std::string DiffDriveLatController::Name() const { return name_; }

void DiffDriveLatController::Stop() { CloseLogFile(); }

}  // namespace control
}  // namespace apollo
