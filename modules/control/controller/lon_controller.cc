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

#include <cstdio>
#include <utility>

#include "modules/common/log.h"
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
    strftime(name_buffer, 80, "/tmp/speed_log__%F_%H%M%S.csv",
             localtime(&rawtime));
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
void LonController::Stop() {
  CloseLogFile();
}

LonController::~LonController() {
  CloseLogFile();
}

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

  station_pid_controller_.Init(lon_controller_conf.station_pid_conf());
  speed_pid_controller_.Init(lon_controller_conf.low_speed_pid_conf());

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

  if (preview_time < 0.0) {
    const auto error_msg = common::util::StrCat(
        "Preview time set as: ", preview_time, " less than 0");
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }
  ComputeLongitudinalErrors(trajectory_analyzer_.get(), preview_time, debug);

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
  double speed_offset =
      station_pid_controller_.Control(station_error_limited, ts);

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
  if (VehicleStateProvider::instance()->linear_velocity() <=
      lon_controller_conf.switch_speed()) {
    speed_pid_controller_.SetPID(lon_controller_conf.low_speed_pid_conf());
    acceleration_cmd_closeloop =
        speed_pid_controller_.Control(speed_controller_input_limited, ts);
  } else {
    speed_pid_controller_.SetPID(lon_controller_conf.high_speed_pid_conf());
    acceleration_cmd_closeloop =
        speed_pid_controller_.Control(speed_controller_input_limited, ts);
  }

  double slope_offset_compenstaion = digital_filter_pitch_angle_.Filter(
      GRA_ACC * std::sin(VehicleStateProvider::instance()->pitch()));

  debug->set_slope_offset_compensation(slope_offset_compenstaion);

  double acceleration_cmd =
      acceleration_cmd_closeloop + debug->preview_acceleration_reference() +
      FLAGS_enable_slope_offset * debug->slope_offset_compensation();
  debug->set_is_full_stop(false);
  if (std::fabs(debug->preview_acceleration_reference()) <=
          FLAGS_max_acceleration_when_stopped &&
      std::fabs(debug->preview_speed_reference()) <=
          FLAGS_max_abs_speed_when_stopped) {
    acceleration_cmd = lon_controller_conf.standstill_acceleration();
    AINFO << "Stop location reached";
    debug->set_is_full_stop(true);
  }

  double throttle_deadzone = lon_controller_conf.throttle_deadzone();
  double brake_deadzone = lon_controller_conf.brake_deadzone();
  double calibration_value = 0.0;
  if (FLAGS_use_preview_speed_for_table) {
    calibration_value = control_interpolation_->Interpolate(
        std::make_pair(debug->preview_speed_reference(), acceleration_cmd));
  } else {
    calibration_value = control_interpolation_->Interpolate(
        std::make_pair(chassis_->speed_mps(), acceleration_cmd));
  }

  if (calibration_value >= 0) {
    throttle_cmd = calibration_value > throttle_deadzone ? calibration_value
                                                         : throttle_deadzone;
    brake_cmd = 0.0;
  } else {
    throttle_cmd = 0.0;
    brake_cmd = -calibration_value > brake_deadzone ? -calibration_value
                                                    : brake_deadzone;
  }

  debug->set_station_error_limited(station_error_limited);
  debug->set_speed_controller_input_limited(speed_controller_input_limited);
  debug->set_acceleration_cmd(acceleration_cmd);
  debug->set_throttle_cmd(throttle_cmd);
  debug->set_brake_cmd(brake_cmd);
  debug->set_acceleration_lookup(acceleration_cmd);
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

  cmd->set_throttle(throttle_cmd);
  cmd->set_brake(brake_cmd);

  if (std::fabs(VehicleStateProvider::instance()->linear_velocity()) <=
          FLAGS_max_abs_speed_when_stopped ||
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

std::string LonController::Name() const {
  return name_;
}

void LonController::ComputeLongitudinalErrors(
    const TrajectoryAnalyzer *trajectory_analyzer, const double preview_time,
    SimpleLongitudinalDebug *debug) {
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
      VehicleStateProvider::instance()->x(),
      VehicleStateProvider::instance()->y());

  trajectory_analyzer->ToTrajectoryFrame(
      VehicleStateProvider::instance()->x(),
      VehicleStateProvider::instance()->y(),
      VehicleStateProvider::instance()->heading(),
      VehicleStateProvider::instance()->linear_velocity(), matched_point,
      &s_matched, &s_dot_matched, &d_matched, &d_dot_matched);

  double current_control_time = Clock::NowInSeconds();
  double preview_control_time = current_control_time + preview_time;

  TrajectoryPoint reference_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(
          current_control_time);
  TrajectoryPoint preview_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(
          preview_control_time);

  ADEBUG << "matched point:" << matched_point.DebugString();
  ADEBUG << "reference point:" << reference_point.DebugString();
  ADEBUG << "preview point:" << preview_point.DebugString();
  debug->set_station_error(reference_point.path_point().s() - s_matched);
  debug->set_speed_error(reference_point.v() - s_dot_matched);

  debug->set_station_reference(reference_point.path_point().s());
  debug->set_speed_reference(reference_point.v());
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

}  // namespace control
}  // namespace apollo
