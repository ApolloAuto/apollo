/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include "modules/control/controller/lon_controller.h"

#include <algorithm>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_util.h"
#include "modules/control/common/control_gflags.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace control {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleStateProvider;
using apollo::common::time::Clock;

constexpr double GRA_ACC = 9.8;

LonController::LonController()
    : name_(ControlConf_ControllerType_Name(ControlConf::LON_CONTROLLER)) {
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
              "preview_acceleration_reference,"
              "acceleration_cmd_closeloop,"
              "acceleration_cmd,"
              "acceleration_lookup,"
              "speed_lookup,"
              "calibration_value,"
              "throttle_cmd,"
              "brake_cmd,"
              "is_full_stop,"
              "\r\n");

      fflush(speed_log_file_);
    }
    AINFO << name_ << " used.";
  }
}

void LonController::CloseLogFile() {
  if (FLAGS_enable_csv_debug) {
    if (speed_log_file_ != nullptr) {
      fclose(speed_log_file_);
      speed_log_file_ = nullptr;
    }
  }
}
void LonController::Stop() { CloseLogFile(); }

LonController::~LonController() { CloseLogFile(); }

Status LonController::Init(const ControlConf *control_conf) {
  control_conf_ = control_conf;
  if (control_conf_ == nullptr) {
    controller_initialized_ = false;
    AERROR << "get_longitudinal_param() nullptr";
    return Status(ErrorCode::CONTROL_INIT_ERROR,
                  "Failed to load LonController conf");
  }
  const LonControllerConf &lon_controller_conf =
      control_conf_->lon_controller_conf();
  double ts = lon_controller_conf.ts();
  bool enable_leadlag =
      lon_controller_conf.enable_reverse_leadlag_compensation();

  station_pid_controller_.Init(lon_controller_conf.station_pid_conf());
  speed_pid_controller_.Init(lon_controller_conf.low_speed_pid_conf());

  if (enable_leadlag) {
    station_leadlag_controller_.Init(
        lon_controller_conf.reverse_station_leadlag_conf(), ts);
    speed_leadlag_controller_.Init(
        lon_controller_conf.reverse_speed_leadlag_conf(), ts);
  }

  vehicle_param_.CopyFrom(
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param());

  SetDigitalFilterPitchAngle(lon_controller_conf);

  LoadControlCalibrationTable(lon_controller_conf);
  controller_initialized_ = true;

  return Status::OK();
}

void LonController::SetDigitalFilterPitchAngle(
    const LonControllerConf &lon_controller_conf) {
  double cutoff_freq =
      lon_controller_conf.pitch_angle_filter_conf().cutoff_freq();
  double ts = lon_controller_conf.ts();
  SetDigitalFilter(ts, cutoff_freq, &digital_filter_pitch_angle_);
}

void LonController::LoadControlCalibrationTable(
    const LonControllerConf &lon_controller_conf) {
  const auto &control_table = lon_controller_conf.calibration_table();
  AINFO << "Control calibration table loaded";
  AINFO << "Control calibration table size is "
        << control_table.calibration_size();
  Interpolation2D::DataType xyz;
  for (const auto &calibration : control_table.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  control_interpolation_.reset(new Interpolation2D);
  CHECK(control_interpolation_->Init(xyz))
      << "Fail to load control calibration table";
}

Status LonController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    control::ControlCommand *cmd) {
  localization_ = localization;
  chassis_ = chassis;

  trajectory_message_ = planning_published_trajectory;
  if (!control_interpolation_) {
    AERROR << "Fail to initialize calibration table.";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "Fail to initialize calibration table.");
  }

  if (trajectory_analyzer_ == nullptr ||
      trajectory_analyzer_->seq_num() !=
          trajectory_message_->header().sequence_num()) {
    trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
  }
  const LonControllerConf &lon_controller_conf =
      control_conf_->lon_controller_conf();

  auto debug = cmd->mutable_debug()->mutable_simple_lon_debug();
  debug->Clear();

  double brake_cmd = 0.0;
  double throttle_cmd = 0.0;
  double ts = lon_controller_conf.ts();
  double preview_time = lon_controller_conf.preview_window() * ts;
  bool enable_leadlag =
      lon_controller_conf.enable_reverse_leadlag_compensation();

  if (preview_time < 0.0) {
    const auto error_msg = common::util::StrCat(
        "Preview time set as: ", preview_time, " less than 0");
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }
  ComputeLongitudinalErrors(trajectory_analyzer_.get(), preview_time, ts,
                            debug);

  double station_error_limit = lon_controller_conf.station_error_limit();
  double station_error_limited = 0.0;
  if (FLAGS_enable_speed_station_preview) {
    station_error_limited =
        common::math::Clamp(debug->preview_station_error(),
                            -station_error_limit, station_error_limit);
  } else {
    station_error_limited = common::math::Clamp(
        debug->station_error(), -station_error_limit, station_error_limit);
  }

  if (trajectory_message_->gear() == canbus::Chassis::GEAR_REVERSE) {
    station_pid_controller_.SetPID(
        lon_controller_conf.reverse_station_pid_conf());
    speed_pid_controller_.SetPID(lon_controller_conf.reverse_speed_pid_conf());
    if (enable_leadlag) {
      station_leadlag_controller_.SetLeadlag(
          lon_controller_conf.reverse_station_leadlag_conf());
      speed_leadlag_controller_.SetLeadlag(
          lon_controller_conf.reverse_speed_leadlag_conf());
    }
  } else if (VehicleStateProvider::Instance()->linear_velocity() <=
             lon_controller_conf.switch_speed()) {
    speed_pid_controller_.SetPID(lon_controller_conf.low_speed_pid_conf());
  } else {
    speed_pid_controller_.SetPID(lon_controller_conf.high_speed_pid_conf());
  }

  double speed_offset =
      station_pid_controller_.Control(station_error_limited, ts);
  if (enable_leadlag) {
    speed_offset = station_leadlag_controller_.Control(speed_offset, ts);
  }

  double speed_controller_input = 0.0;
  double speed_controller_input_limit =
      lon_controller_conf.speed_controller_input_limit();
  double speed_controller_input_limited = 0.0;
  if (FLAGS_enable_speed_station_preview) {
    speed_controller_input = speed_offset + debug->preview_speed_error();
  } else {
    speed_controller_input = speed_offset + debug->speed_error();
  }
  speed_controller_input_limited =
      common::math::Clamp(speed_controller_input, -speed_controller_input_limit,
                          speed_controller_input_limit);

  double acceleration_cmd_closeloop = 0.0;

  acceleration_cmd_closeloop =
      speed_pid_controller_.Control(speed_controller_input_limited, ts);
  debug->set_pid_saturation_status(
      speed_pid_controller_.IntegratorSaturationStatus());
  if (enable_leadlag) {
    acceleration_cmd_closeloop =
        speed_leadlag_controller_.Control(acceleration_cmd_closeloop, ts);
    debug->set_leadlag_saturation_status(
        speed_leadlag_controller_.InnerstateSaturationStatus());
  }

  double slope_offset_compenstaion = digital_filter_pitch_angle_.Filter(
      GRA_ACC * std::sin(VehicleStateProvider::Instance()->pitch()));

  if (std::isnan(slope_offset_compenstaion)) {
    slope_offset_compenstaion = 0;
  }

  debug->set_slope_offset_compensation(slope_offset_compenstaion);

  double acceleration_cmd =
      acceleration_cmd_closeloop + debug->preview_acceleration_reference() +
      FLAGS_enable_slope_offset * debug->slope_offset_compensation();
  debug->set_is_full_stop(false);
  GetPathRemain(debug);

  // At near-stop stage, replace the brake control command with the standstill
  // accleration if the former is even softer than the latter
  if ((trajectory_message_->trajectory_type() ==
       apollo::planning::ADCTrajectory::NORMAL) &&
      ((std::fabs(debug->preview_acceleration_reference()) <=
            control_conf_->max_acceleration_when_stopped() &&
        std::fabs(debug->preview_speed_reference()) <=
            vehicle_param_.max_abs_speed_when_stopped()) ||
       std::abs(debug->path_remain()) <
           control_conf_->max_path_remain_when_stopped())) {
    acceleration_cmd =
        (chassis->gear_location() == canbus::Chassis::GEAR_REVERSE)
            ? std::max(acceleration_cmd,
                       -lon_controller_conf.standstill_acceleration())
            : std::min(acceleration_cmd,
                       lon_controller_conf.standstill_acceleration());
    ADEBUG << "Stop location reached";
    debug->set_is_full_stop(true);
  }

  double throttle_lowerbound =
      std::max(vehicle_param_.throttle_deadzone(),
               lon_controller_conf.throttle_minimum_action());
  double brake_lowerbound =
      std::max(vehicle_param_.brake_deadzone(),
               lon_controller_conf.brake_minimum_action());
  double calibration_value = 0.0;
  double acceleration_lookup =
      (chassis->gear_location() == canbus::Chassis::GEAR_REVERSE)
          ? -acceleration_cmd
          : acceleration_cmd;

  if (FLAGS_use_preview_speed_for_table) {
    calibration_value = control_interpolation_->Interpolate(
        std::make_pair(debug->preview_speed_reference(), acceleration_lookup));
  } else {
    calibration_value = control_interpolation_->Interpolate(
        std::make_pair(chassis_->speed_mps(), acceleration_lookup));
  }

  if (acceleration_lookup >= 0) {
    if (calibration_value >= 0) {
      throttle_cmd = std::max(calibration_value, throttle_lowerbound);
    } else {
      throttle_cmd = throttle_lowerbound;
    }
    brake_cmd = 0.0;
  } else {
    throttle_cmd = 0.0;
    if (calibration_value >= 0) {
      brake_cmd = brake_lowerbound;
    } else {
      brake_cmd = std::max(-calibration_value, brake_lowerbound);
    }
  }

  debug->set_station_error_limited(station_error_limited);
  debug->set_speed_offset(speed_offset);
  debug->set_speed_controller_input_limited(speed_controller_input_limited);
  debug->set_acceleration_cmd(acceleration_cmd);
  debug->set_throttle_cmd(throttle_cmd);
  debug->set_brake_cmd(brake_cmd);
  debug->set_acceleration_lookup(acceleration_lookup);
  debug->set_speed_lookup(chassis_->speed_mps());
  debug->set_calibration_value(calibration_value);
  debug->set_acceleration_cmd_closeloop(acceleration_cmd_closeloop);

  if (FLAGS_enable_csv_debug && speed_log_file_ != nullptr) {
    fprintf(speed_log_file_,
            "%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f,"
            "%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %d,\r\n",
            debug->station_reference(), debug->station_error(),
            station_error_limited, debug->preview_station_error(),
            debug->speed_reference(), debug->speed_error(),
            speed_controller_input_limited, debug->preview_speed_reference(),
            debug->preview_speed_error(),
            debug->preview_acceleration_reference(), acceleration_cmd_closeloop,
            acceleration_cmd, debug->acceleration_lookup(),
            debug->speed_lookup(), calibration_value, throttle_cmd, brake_cmd,
            debug->is_full_stop());
  }

  // if the car is driven by acceleration, disgard the cmd->throttle and brake
  cmd->set_throttle(throttle_cmd);
  cmd->set_brake(brake_cmd);
  cmd->set_acceleration(acceleration_cmd);

  if (std::fabs(VehicleStateProvider::Instance()->linear_velocity()) <=
          vehicle_param_.max_abs_speed_when_stopped() ||
      chassis->gear_location() == trajectory_message_->gear() ||
      chassis->gear_location() == canbus::Chassis::GEAR_NEUTRAL) {
    cmd->set_gear_location(trajectory_message_->gear());
  } else {
    cmd->set_gear_location(chassis->gear_location());
  }

  return Status::OK();
}

Status LonController::Reset() {
  speed_pid_controller_.Reset();
  station_pid_controller_.Reset();
  return Status::OK();
}

std::string LonController::Name() const { return name_; }

void LonController::ComputeLongitudinalErrors(
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

  auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
      VehicleStateProvider::Instance()->x(),
      VehicleStateProvider::Instance()->y());

  trajectory_analyzer->ToTrajectoryFrame(
      VehicleStateProvider::Instance()->x(),
      VehicleStateProvider::Instance()->y(),
      VehicleStateProvider::Instance()->heading(),
      VehicleStateProvider::Instance()->linear_velocity(), matched_point,
      &s_matched, &s_dot_matched, &d_matched, &d_dot_matched);

  double current_control_time = Clock::NowInSeconds();
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

  double heading_error = common::math::NormalizeAngle(
      VehicleStateProvider::Instance()->heading() - matched_point.theta());
  double lon_speed = VehicleStateProvider::Instance()->linear_velocity() *
                     std::cos(heading_error);
  double lon_acceleration =
      VehicleStateProvider::Instance()->linear_acceleration() *
      std::cos(heading_error);
  double one_minus_kappa_lat_error =
      1 - reference_point.path_point().kappa() *
              VehicleStateProvider::Instance()->linear_velocity() *
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

void LonController::SetDigitalFilter(double ts, double cutoff_freq,
                                     common::DigitalFilter *digital_filter) {
  std::vector<double> denominators;
  std::vector<double> numerators;
  common::LpfCoefficients(ts, cutoff_freq, &denominators, &numerators);
  digital_filter->set_coefficients(denominators, numerators);
}

// TODO(all): Refactor and simplify
void LonController::GetPathRemain(SimpleLongitudinalDebug *debug) {
  int stop_index = 0;

  if (trajectory_message_->gear() == canbus::Chassis::GEAR_DRIVE) {
    while (stop_index < trajectory_message_->trajectory_point_size()) {
      if (fabs(trajectory_message_->trajectory_point(stop_index).v()) < 1e-3 &&
          trajectory_message_->trajectory_point(stop_index).a() > -0.01 &&
          trajectory_message_->trajectory_point(stop_index).a() < 0.0) {
        break;
      } else {
        ++stop_index;
      }
    }
  } else {
    while (stop_index < trajectory_message_->trajectory_point_size()) {
      if (fabs(trajectory_message_->trajectory_point(stop_index).v()) < 1e-3 &&
          trajectory_message_->trajectory_point(stop_index).a() < 0.1 &&
          trajectory_message_->trajectory_point(stop_index).a() > 0.0) {
        break;
      } else {
        ++stop_index;
      }
    }
  }
  if (stop_index == trajectory_message_->trajectory_point_size()) {
    --stop_index;
    if (fabs(trajectory_message_->trajectory_point(stop_index).v()) < 0.1) {
      ADEBUG << "the last point is selected as parking point";
    } else {
      ADEBUG << "the last point found in path and speed > speed_deadzone";
      debug->set_path_remain(10000);
    }
  }
  debug->set_path_remain(
      trajectory_message_->trajectory_point(stop_index).path_point().s() -
      debug->current_station());
}

}  // namespace control
}  // namespace apollo
