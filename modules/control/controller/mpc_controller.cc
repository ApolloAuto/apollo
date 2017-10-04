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

#include "modules/control/controller/mpc_controller.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/LU"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/mpc_solver.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_util.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace control {

using common::TrajectoryPoint;
using common::Point3D;
using common::VehicleState;
using Matrix = Eigen::MatrixXd;
using apollo::common::Status;
using apollo::common::ErrorCode;
using apollo::common::time::Clock;

namespace {

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  strftime(name_buffer, 80, "/tmp/steer_log_simple_optimal_%F_%H%M%S.csv",
           localtime(&raw_time));
  return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream) {
  file_stream << "current_lateral_error,"
              << "current_ref_heading,"
              << "current_heading,"
              << "current_heading_error,"
              << "heading_error_rate,"
              << "lateral_error_rate,"
              << "current_curvature,"
              << "steer_angle,"
              << "steer_angle_feedforward,"
              << "steer_angle_lateral_contribution,"
              << "steer_angle_lateral_rate_contribution,"
              << "steer_angle_heading_contribution,"
              << "steer_angle_heading_rate_contribution,"
              << "steer_angle_feedback,"
              << "steering_position,"
              << "v" << std::endl;
}
}  // namespace

MPCController::MPCController() : name_("LQR-based Lateral Controller") {
  if (FLAGS_enable_csv_debug) {
    steer_log_file_.open(GetLogFileName());
    steer_log_file_ << std::fixed;
    steer_log_file_ << std::setprecision(6);
    WriteHeaders(steer_log_file_);
  }
  AINFO << "Using " << name_;
}

MPCController::~MPCController() {
  CloseLogFile();
}

bool MPCController::LoadControlConf(const ControlConf *control_conf) {
  if (!control_conf) {
    AERROR << "[MPCController] control_conf == nullptr";
    return false;
  }
  const auto &vehicle_param_ =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();

  ts_ = control_conf->mpc_controller_conf().ts();
  CHECK_GT(ts_, 0.0) << "[MPCController] Invalid control update interval.";
  cf_ = control_conf->mpc_controller_conf().cf();
  cr_ = control_conf->mpc_controller_conf().cr();
  wheelbase_ = vehicle_param_.wheel_base();
  steer_transmission_ratio_ = vehicle_param_.steer_ratio();
  steer_single_direction_max_degree_ =
      vehicle_param_.max_steer_angle() / M_PI * 180;
  max_lat_acc_ = control_conf->mpc_controller_conf().max_lateral_acceleration();

  double mass_fl = control_conf->mpc_controller_conf().mass_fl();
  double mass_fr = control_conf->mpc_controller_conf().mass_fr();
  double mass_rl = control_conf->mpc_controller_conf().mass_rl();
  double mass_rr = control_conf->mpc_controller_conf().mass_rr();
  double mass_front = mass_fl + mass_fr;
  double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  lqr_eps_ = control_conf->mpc_controller_conf().eps();
  lqr_max_iteration_ = control_conf->mpc_controller_conf().max_iteration();
  throttle_deadzone_ = control_conf->mpc_controller_conf().throttle_deadzone();
  brake_deadzone_ = control_conf->mpc_controller_conf().brake_deadzone();

  LoadControlCalibrationTable(control_conf->mpc_controller_conf());
  return true;
}

void MPCController::ProcessLogs(const SimpleMPCDebug *debug,
                                const canbus::Chassis *chassis) {
  const std::string log_str = apollo::common::util::StrCat(
      debug->lateral_error(), ",", debug->ref_heading(), ",",
      VehicleState::instance()->heading(), ",", debug->heading_error(), ",",
      debug->heading_error_rate(), ",", debug->lateral_error_rate(), ",",
      debug->curvature(), ",", debug->steer_angle(), ",",
      debug->steer_angle_feedforward(), ",",
      debug->steer_angle_lateral_contribution(), ",",
      debug->steer_angle_lateral_rate_contribution(), ",",
      debug->steer_angle_heading_contribution(), ",",
      debug->steer_angle_heading_rate_contribution(), ",",
      debug->steer_angle_feedback(), ",", chassis->steering_percentage(), ",",
      VehicleState::instance()->linear_velocity());
  if (FLAGS_enable_csv_debug) {
    steer_log_file_ << log_str << std::endl;
  }
  ADEBUG << "Steer_Control_Detail: " << log_str;
}

void MPCController::LogInitParameters() {
  AINFO << name_ << " begin.";
  AINFO << "[MPCController parameters]"
        << " mass_: " << mass_ << ","
        << " iz_: " << iz_ << ","
        << " lf_: " << lf_ << ","
        << " lr_: " << lr_;
}

void MPCController::InitializeFilters(const ControlConf *control_conf) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  LpfCoefficients(ts_, control_conf->mpc_controller_conf().cutoff_freq(), &den,
                  &num);
  digital_filter_.set_coefficients(den, num);
  // Mean filters
  /**
  heading_rate_filter_ = MeanFilter(
      control_conf->mpc_controller_conf().mean_filter_window_size());
  **/
  lateral_error_filter_ =
      MeanFilter(control_conf->mpc_controller_conf().mean_filter_window_size());
}

Status MPCController::Init(const ControlConf *control_conf) {
  if (!LoadControlConf(control_conf)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }
  // Matrix init operations.
  int matrix_size = basic_state_size_;
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_(4, 4) = 1.0;
  matrix_a_(5, 5) = 1.0; //TODO: change to add delays

  matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(2, 3) = 1.0;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  matrix_b_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;

  matrix_state_ = Matrix::Zero(matrix_size, 1);
  matrix_k_ = Matrix::Zero(1, matrix_size);
  // TODO: Change it to configs
  matrix_r_ = Matrix::Identity(2, 2);
  matrix_q_ = Matrix::Zero(matrix_size, matrix_size);

  int r_param_size = control_conf->mpc_controller_conf().matrix_r_size();
  for (int i = 0; i < r_param_size; ++i) {
    matrix_r_(i, i) = control_conf->mpc_controller_conf().matrix_r(i);
  }

  int q_param_size = control_conf->mpc_controller_conf().matrix_q_size();
  if (matrix_size != q_param_size) {
    const auto error_msg = apollo::common::util::StrCat(
        "MPC controller error: matrix_q size: ", q_param_size,
        " in parameter file not equal to matrix_size: ", matrix_size);
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }
  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf->mpc_controller_conf().matrix_q(i);
  }

  matrix_q_updated_ = matrix_q_;
  InitializeFilters(control_conf);
  LogInitParameters();
  return Status::OK();
}

void MPCController::CloseLogFile() {
  if (FLAGS_enable_csv_debug && steer_log_file_.is_open()) {
    steer_log_file_.close();
  }
}

void MPCController::Stop() {
  CloseLogFile();
}

std::string MPCController::Name() const {
  return name_;
}

Status MPCController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    ControlCommand *cmd) {
  VehicleState::instance()->set_linear_velocity(
      std::max(VehicleState::instance()->linear_velocity(), 1.0));

  trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(planning_published_trajectory));

  SimpleMPCDebug *debug = cmd->mutable_debug()->mutable_simple_mpc_debug();
  debug->Clear();

  // Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading
  // Error Rate.
  UpdateStateAnalyticalMatching(debug);

  UpdateMatrix();

int CONTROLS = 2;
const int HORIZON = 10;

Eigen::MatrixXd control_matrix(CONTROLS, 1);
control_matrix << 0,
                  0;

                  Eigen::MatrixXd C(basic_state_size_, 1);
                  C << 0,
                       0,
                       0,
                       0,
                       0,
                       0;

Eigen::MatrixXd reference_state(basic_state_size_, 1);
reference_state << 0,
                   0,
                   0,
                   0,
                   0,
                   0;

std::vector<Eigen::MatrixXd> reference(HORIZON, reference_state);


                  Eigen::MatrixXd lower_bound(CONTROLS, 1);
                  lower_bound << -10,
                                 -10;

                  Eigen::MatrixXd upper_bound(CONTROLS, 1);
                  upper_bound << 10,
                                 10;

                                 Eigen::MatrixXd initial_state(basic_state_size_, 1);
                                 initial_state << 0,
                                                  0,
                                                  0,
                                                  0,
                                                  0,
                                                  0;
  std::vector<Eigen::MatrixXd> control(HORIZON, control_matrix);
    ::apollo::common::math::SolveLinearMPC(matrix_ad_, matrix_bd_, C, matrix_q_, matrix_r_, lower_bound, upper_bound, initial_state,
                   reference, lqr_eps_, lqr_max_iteration_, &control);

    AERROR << "I am still alive !";
  double steer_angle_feedback = -control[0](1, 0);
  AERROR << "steer passed !";
  double steer_angle_feedforward = ComputeLateralFeedForward(debug->curvature());
  double steer_angle = steer_angle_feedback + steer_angle_feedforward;

  // Clamp the steer angle to -100.0 to 100.0
  steer_angle = apollo::common::math::Clamp(steer_angle, -100.0, 100.0);

  if (FLAGS_set_steer_limit) {
    double steer_limit =
        std::atan(max_lat_acc_ * wheelbase_ /
                  (VehicleState::instance()->linear_velocity() *
                   VehicleState::instance()->linear_velocity())) *
        steer_transmission_ratio_ * 180 / M_PI /
        steer_single_direction_max_degree_ * 100;

    // Clamp the steer angle
    double steer_angle_limited =
        apollo::common::math::Clamp(steer_angle, -steer_limit, steer_limit);
    steer_angle_limited = digital_filter_.Filter(steer_angle_limited);
    cmd->set_steering_target(steer_angle_limited);
    debug->set_steer_angle_limited(steer_angle_limited);
  } else {
    steer_angle = digital_filter_.Filter(steer_angle);
    cmd->set_steering_target(steer_angle);
  }

  double acceleration_cmd =
      -control[0](0, 0) + debug->acceleration_reference(); //TODO: add pitch angle feedforward
  debug->set_is_full_stop(false);
  if (std::abs(debug->acceleration_reference()) <=
          FLAGS_max_acceleration_when_stopped &&
      std::abs(debug->speed_reference()) <=
          FLAGS_max_abs_speed_when_stopped) {
    acceleration_cmd = standstill_acceleration_;
    AINFO << "Stop location reached";
    debug->set_is_full_stop(true);
  }
  AERROR << "acceleration passed !";
  double calibration_value = 0.0;
  if (FLAGS_use_preview_speed_for_table) {
    calibration_value = control_interpolation_->Interpolate(
        std::make_pair(debug->speed_reference(), acceleration_cmd));
  } else {
    calibration_value = control_interpolation_->Interpolate(
        std::make_pair(VehicleState::instance()->linear_velocity(), acceleration_cmd));
  }

  double throttle_cmd = 0.0;
  double brake_cmd = 0.0;
  if (calibration_value >= 0) {
    throttle_cmd = calibration_value > throttle_deadzone_ ? calibration_value
                                                         : throttle_deadzone_;
    brake_cmd = 0.0;
  } else {
    throttle_cmd = 0.0;
    brake_cmd = -calibration_value > brake_deadzone_ ? -calibration_value
                                                    : brake_deadzone_;
  }


  cmd->set_steering_rate(FLAGS_steer_angle_rate);
  cmd->set_throttle(throttle_cmd);
  cmd->set_brake(brake_cmd);
  // compute extra information for logging and debugging
  double steer_angle_lateral_contribution =
      -matrix_k_(0, 0) * matrix_state_(0, 0) * 180 / M_PI *
      steer_transmission_ratio_ / steer_single_direction_max_degree_ * 100;

  double steer_angle_lateral_rate_contribution =
      -matrix_k_(0, 1) * matrix_state_(1, 0) * 180 / M_PI *
      steer_transmission_ratio_ / steer_single_direction_max_degree_ * 100;

  double steer_angle_heading_contribution =
      -matrix_k_(0, 2) * matrix_state_(2, 0) * 180 / M_PI *
      steer_transmission_ratio_ / steer_single_direction_max_degree_ * 100;

  double steer_angle_heading_rate_contribution =
      -matrix_k_(0, 3) * matrix_state_(3, 0) * 180 / M_PI *
      steer_transmission_ratio_ / steer_single_direction_max_degree_ * 100;

  debug->set_heading(VehicleState::instance()->heading());
  debug->set_steer_angle(steer_angle);
  debug->set_steer_angle_feedforward(steer_angle_feedforward);
  debug->set_steer_angle_lateral_contribution(steer_angle_lateral_contribution);
  debug->set_steer_angle_lateral_rate_contribution(
      steer_angle_lateral_rate_contribution);
  debug->set_steer_angle_heading_contribution(steer_angle_heading_contribution);
  debug->set_steer_angle_heading_rate_contribution(
      steer_angle_heading_rate_contribution);
  debug->set_steer_angle_feedback(steer_angle_feedback);
  debug->set_steering_position(chassis->steering_percentage());

  ProcessLogs(debug, chassis);
  return Status::OK();
}

Status MPCController::Reset() {
  previous_heading_error_ = 0.0;
  previous_lateral_error_ = 0.0;
  return Status::OK();
}

void MPCController::LoadControlCalibrationTable(
    const MPCControllerConf &mpc_controller_conf) {
  const auto &control_table = mpc_controller_conf.calibration_table();
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

// state = [Lateral Error, Lateral Error Rate, Heading Error, Heading Error
// Rate, Preview Lateral1, Preview Lateral2, ...]
void MPCController::UpdateState(SimpleMPCDebug *debug) {
  TrajectoryPoint traj_point;
  const auto &position = VehicleState::instance()->ComputeCOMPosition(lr_);
  double raw_lateral_error = GetLateralError(position, &traj_point);

  // lateral_error_ = lateral_rate_filter_.Filter(raw_lateral_error);
  debug->set_lateral_error(lateral_error_filter_.Update(raw_lateral_error));

  // ref_curvature_ = traj_point.kappa();
  debug->set_curvature(traj_point.path_point().kappa());

  // ref_heading_ = traj_point.theta;
  debug->set_ref_heading(traj_point.path_point().theta());

  // heading_error_ =
  //    common::math::NormalizeAngle(VehicleState::instance()->heading() -
  //    ref_heading_);
  debug->set_heading_error(common::math::NormalizeAngle(
      VehicleState::instance()->heading() - traj_point.path_point().theta()));

  // Reverse heading error if vehicle is going in reverse
  if (VehicleState::instance()->gear() ==
      canbus::Chassis::GEAR_REVERSE) {
    debug->set_heading_error(-debug->heading_error());
  }

  // heading_error_rate_ = (heading_error_ - previous_heading_error_) / ts_;
  debug->set_heading_error_rate(
      (debug->heading_error() - previous_heading_error_) / ts_);
  // lateral_error_rate_ = (lateral_error_ - previous_lateral_error_) / ts_;
  debug->set_lateral_error_rate(
      (debug->lateral_error() - previous_lateral_error_) / ts_);

  // Prepare for next iteration.
  previous_heading_error_ = debug->heading_error();
  previous_lateral_error_ = debug->lateral_error();

  // State matrix update;
  // First six elements are fixed;
  matrix_state_(0, 0) = debug->lateral_error();
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(2, 0) = debug->heading_error();
  matrix_state_(3, 0) = debug->heading_error_rate();
  matrix_state_(4, 0) = debug->station_error();
  matrix_state_(5, 0) = debug->speed_error();
}

void MPCController::UpdateStateAnalyticalMatching(SimpleMPCDebug *debug) {
  const auto &com = VehicleState::instance()->ComputeCOMPosition(lr_);
  ComputeLateralErrors(com.x(), com.y(), VehicleState::instance()->heading(),
                       VehicleState::instance()->linear_velocity(),
                       VehicleState::instance()->angular_velocity(),
                       trajectory_analyzer_, debug);

  // Reverse heading error if vehicle is going in reverse
  if (VehicleState::instance()->gear() ==
      canbus::Chassis::GEAR_REVERSE) {
    debug->set_heading_error(-debug->heading_error());
  }

  // State matrix update;
  // First four elements are fixed;
  matrix_state_(0, 0) = debug->lateral_error();
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(2, 0) = debug->heading_error();
  matrix_state_(3, 0) = debug->heading_error_rate();

}

void MPCController::UpdateMatrix() {
  double v = VehicleState::instance()->linear_velocity();
  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;

  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  matrix_ad_ = (matrix_i + ts_ * 0.5 * matrix_a_) *
               (matrix_i - ts_ * 0.5 * matrix_a_).inverse();
}

double MPCController::ComputeLateralFeedForward(double ref_curvature) const {
  double kv =
      lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;

  // then change it from rad to %
  double v = VehicleState::instance()->linear_velocity();
  double steer_angle_feedforwardterm =
      (wheelbase_ * ref_curvature + kv * v * v * ref_curvature -
       matrix_k_(0, 2) *
           (lr_ * ref_curvature -
            lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_)) *
      180 / M_PI * steer_transmission_ratio_ /
      steer_single_direction_max_degree_ * 100;
  return steer_angle_feedforwardterm;
}

/*
 * SL coordinate system:
 *  left to the ref_line, L is +
 * right to the ref_line, L is -
 */
double MPCController::GetLateralError(const common::math::Vec2d &point,
                                      TrajectoryPoint *traj_point) const {
  auto closest =
      trajectory_analyzer_.QueryNearestPointByPosition(point.x(), point.y());

  double point_angle = std::atan2(point.y() - closest.path_point().y(),
                                  point.x() - closest.path_point().x());
  double point2path_angle = point_angle - closest.path_point().theta();
  if (traj_point != nullptr) {
    *traj_point = closest;
  }

  double dx = closest.path_point().x() - point.x();
  double dy = closest.path_point().y() - point.y();
  return std::sin(point2path_angle) * std::sqrt(dx * dx + dy * dy);
}

void MPCController::ComputeLateralErrors(
    const double x, const double y, const double theta, const double linear_v,
    const double angular_v, const TrajectoryAnalyzer &trajectory_analyzer,
    SimpleMPCDebug *debug) const {
  auto matched_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);

  double dx = x - matched_point.path_point().x();
  double dy = y - matched_point.path_point().y();

  double cos_matched_theta = std::cos(matched_point.path_point().theta());
  double sin_matched_theta = std::sin(matched_point.path_point().theta());
  // d_error = cos_matched_theta * dy - sin_matched_theta * dx;
  debug->set_lateral_error(cos_matched_theta * dy - sin_matched_theta * dx);

  double delta_theta =
      common::math::NormalizeAngle(theta - matched_point.path_point().theta());
  double sin_delta_theta = std::sin(delta_theta);
  // d_error_dot = chassis_v * sin_delta_theta;
  debug->set_lateral_error_rate(linear_v * sin_delta_theta);

  // theta_error = delta_theta;
  debug->set_heading_error(delta_theta);
  // theta_error_dot = angular_v - matched_point.path_point().kappa() *
  // matched_point.v();
  debug->set_heading_error_rate(angular_v - matched_point.path_point().kappa() *
                                                matched_point.v());

  // matched_theta = matched_point.path_point().theta();
  debug->set_ref_heading(matched_point.path_point().theta());
  // matched_kappa = matched_point.path_point().kappa();
  debug->set_curvature(matched_point.path_point().kappa());
}

  void MPCController::ComputeLongitudinalErrors(
    const TrajectoryAnalyzer *trajectory_analyzer,
    SimpleMPCDebug *debug) {
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
      VehicleState::instance()->x(), VehicleState::instance()->y());

  trajectory_analyzer->ToTrajectoryFrame(
      VehicleState::instance()->x(), VehicleState::instance()->y(),
      VehicleState::instance()->heading(),
      VehicleState::instance()->linear_velocity(), matched_point, &s_matched,
      &s_dot_matched, &d_matched, &d_dot_matched);

  double current_control_time = Clock::NowInSecond();

  TrajectoryPoint reference_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(
          current_control_time);

  ADEBUG << "matched point:" << matched_point.DebugString();
  ADEBUG << "reference point:" << reference_point.DebugString();
  debug->set_station_error(reference_point.path_point().s() - s_matched);
  debug->set_speed_error(reference_point.v() - s_dot_matched);

  debug->set_station_reference(reference_point.path_point().s());
  debug->set_speed_reference(reference_point.v());
  debug->set_acceleration_reference(reference_point.a());
}

}  // namespace control
}  // namespace apollo
