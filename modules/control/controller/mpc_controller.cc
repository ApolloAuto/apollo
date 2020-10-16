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
#include <iomanip>
#include <limits>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "absl/strings/str_cat.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/mpc_osqp.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace control {
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleStateProvider;
using apollo::cyber::Clock;
using Matrix = Eigen::MatrixXd;
using apollo::common::VehicleConfigHelper;

namespace {

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);

  std::tm time_tm;
  localtime_r(&raw_time, &time_tm);
  strftime(name_buffer, sizeof(name_buffer),
           "/tmp/mpc_controller_%F_%H%M%S.csv", &time_tm);
  return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream) {}
}  // namespace

MPCController::MPCController() : name_("MPC Controller") {
  if (FLAGS_enable_csv_debug) {
    mpc_log_file_.open(GetLogFileName());
    mpc_log_file_ << std::fixed;
    mpc_log_file_ << std::setprecision(6);
    WriteHeaders(mpc_log_file_);
  }
  AINFO << "Using " << name_;
}

MPCController::~MPCController() { CloseLogFile(); }

bool MPCController::LoadControlConf(const ControlConf *control_conf) {
  if (!control_conf) {
    AERROR << "[MPCController] control_conf = nullptr";
    return false;
  }
  vehicle_param_ = VehicleConfigHelper::Instance()->GetConfig().vehicle_param();

  ts_ = control_conf->mpc_controller_conf().ts();
  if (ts_ <= 0.0) {
    AERROR << "[MPCController] Invalid control update interval.";
    return false;
  }
  cf_ = control_conf->mpc_controller_conf().cf();
  cr_ = control_conf->mpc_controller_conf().cr();
  wheelbase_ = vehicle_param_.wheel_base();
  steer_ratio_ = vehicle_param_.steer_ratio();
  steer_single_direction_max_degree_ =
      vehicle_param_.max_steer_angle() * 180 / M_PI;
  max_lat_acc_ = control_conf->mpc_controller_conf().max_lateral_acceleration();

  // TODO(Shu, Qi, Yu): add sanity check for conf values
  // steering ratio should be positive
  static constexpr double kEpsilon = 1e-6;
  if (std::isnan(steer_ratio_) || steer_ratio_ < kEpsilon) {
    AERROR << "[MPCController] steer_ratio = 0";
    return false;
  }
  wheel_single_direction_max_degree_ =
      steer_single_direction_max_degree_ / steer_ratio_ / 180 * M_PI;
  max_acceleration_ = vehicle_param_.max_acceleration();
  max_deceleration_ = vehicle_param_.max_deceleration();

  const double mass_fl = control_conf->mpc_controller_conf().mass_fl();
  const double mass_fr = control_conf->mpc_controller_conf().mass_fr();
  const double mass_rl = control_conf->mpc_controller_conf().mass_rl();
  const double mass_rr = control_conf->mpc_controller_conf().mass_rr();
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  mpc_eps_ = control_conf->mpc_controller_conf().eps();
  mpc_max_iteration_ = control_conf->mpc_controller_conf().max_iteration();
  throttle_lowerbound_ =
      std::max(vehicle_param_.throttle_deadzone(),
               control_conf->mpc_controller_conf().throttle_minimum_action());
  brake_lowerbound_ =
      std::max(vehicle_param_.brake_deadzone(),
               control_conf->mpc_controller_conf().brake_minimum_action());

  minimum_speed_protection_ = control_conf->minimum_speed_protection();
  max_acceleration_when_stopped_ =
      control_conf->max_acceleration_when_stopped();
  max_abs_speed_when_stopped_ = vehicle_param_.max_abs_speed_when_stopped();
  standstill_acceleration_ =
      control_conf->mpc_controller_conf().standstill_acceleration();

  enable_mpc_feedforward_compensation_ =
      control_conf->mpc_controller_conf().enable_mpc_feedforward_compensation();

  unconstrained_control_diff_limit_ =
      control_conf->mpc_controller_conf().unconstrained_control_diff_limit();

  LoadControlCalibrationTable(control_conf->mpc_controller_conf());
  ADEBUG << "MPC conf loaded";
  return true;
}

void MPCController::ProcessLogs(const SimpleMPCDebug *debug,
                                const canbus::Chassis *chassis) {
  // TODO(QiL): Add debug information
}

void MPCController::LogInitParameters() {
  ADEBUG << name_ << " begin.";
  ADEBUG << "[MPCController parameters]"
         << " mass_: " << mass_ << ","
         << " iz_: " << iz_ << ","
         << " lf_: " << lf_ << ","
         << " lr_: " << lr_;
}

void MPCController::InitializeFilters(const ControlConf *control_conf) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpfCoefficients(
      ts_, control_conf->mpc_controller_conf().cutoff_freq(), &den, &num);
  digital_filter_.set_coefficients(den, num);
  lateral_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->mpc_controller_conf().mean_filter_window_size()));
  heading_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->mpc_controller_conf().mean_filter_window_size()));
}

Status MPCController::Init(std::shared_ptr<DependencyInjector> injector,
                           const ControlConf *control_conf) {
  if (!LoadControlConf(control_conf)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }
  injector_ = injector;
  // Matrix init operations.
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_(4, 5) = 1.0;
  matrix_a_(5, 5) = 0.0;
  // TODO(QiL): expand the model to accommodate more combined states.

  matrix_a_coeff_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(2, 3) = 1.0;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  matrix_b_ = Matrix::Zero(basic_state_size_, controls_);
  matrix_bd_ = Matrix::Zero(basic_state_size_, controls_);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_b_(4, 1) = 0.0;
  matrix_b_(5, 1) = -1.0;
  matrix_bd_ = matrix_b_ * ts_;

  matrix_c_ = Matrix::Zero(basic_state_size_, 1);
  matrix_cd_ = Matrix::Zero(basic_state_size_, 1);

  matrix_state_ = Matrix::Zero(basic_state_size_, 1);
  matrix_k_ = Matrix::Zero(1, basic_state_size_);

  matrix_r_ = Matrix::Identity(controls_, controls_);

  matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);

  int r_param_size = control_conf->mpc_controller_conf().matrix_r_size();
  for (int i = 0; i < r_param_size; ++i) {
    matrix_r_(i, i) = control_conf->mpc_controller_conf().matrix_r(i);
  }

  int q_param_size = control_conf->mpc_controller_conf().matrix_q_size();
  if (basic_state_size_ != q_param_size) {
    const auto error_msg =
        absl::StrCat("MPC controller error: matrix_q size: ", q_param_size,
                     " in parameter file not equal to basic_state_size_: ",
                     basic_state_size_);
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }
  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf->mpc_controller_conf().matrix_q(i);
  }

  // Update matrix_q_updated_ and matrix_r_updated_
  matrix_r_updated_ = matrix_r_;
  matrix_q_updated_ = matrix_q_;

  InitializeFilters(control_conf);
  LoadMPCGainScheduler(control_conf->mpc_controller_conf());
  LogInitParameters();
  ADEBUG << "[MPCController] init done!";
  return Status::OK();
}

void MPCController::CloseLogFile() {
  if (FLAGS_enable_csv_debug && mpc_log_file_.is_open()) {
    mpc_log_file_.close();
  }
}

double MPCController::Wheel2SteerPct(const double wheel_angle) {
  return wheel_angle / wheel_single_direction_max_degree_ * 100;
}

void MPCController::Stop() { CloseLogFile(); }

std::string MPCController::Name() const { return name_; }

void MPCController::LoadMPCGainScheduler(
    const MPCControllerConf &mpc_controller_conf) {
  const auto &lat_err_gain_scheduler =
      mpc_controller_conf.lat_err_gain_scheduler();
  const auto &heading_err_gain_scheduler =
      mpc_controller_conf.heading_err_gain_scheduler();
  const auto &feedforwardterm_gain_scheduler =
      mpc_controller_conf.feedforwardterm_gain_scheduler();
  const auto &steer_weight_gain_scheduler =
      mpc_controller_conf.steer_weight_gain_scheduler();
  ADEBUG << "MPC control gain scheduler loaded";
  Interpolation1D::DataType xy1, xy2, xy3, xy4;
  for (const auto &scheduler : lat_err_gain_scheduler.scheduler()) {
    xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : heading_err_gain_scheduler.scheduler()) {
    xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : feedforwardterm_gain_scheduler.scheduler()) {
    xy3.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : steer_weight_gain_scheduler.scheduler()) {
    xy4.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }

  lat_err_interpolation_.reset(new Interpolation1D);
  ACHECK(lat_err_interpolation_->Init(xy1))
      << "Fail to load lateral error gain scheduler for MPC controller";

  heading_err_interpolation_.reset(new Interpolation1D);
  ACHECK(heading_err_interpolation_->Init(xy2))
      << "Fail to load heading error gain scheduler for MPC controller";

  feedforwardterm_interpolation_.reset(new Interpolation1D);
  ACHECK(feedforwardterm_interpolation_->Init(xy3))
      << "Fail to load feed forward term gain scheduler for MPC controller";

  steer_weight_interpolation_.reset(new Interpolation1D);
  ACHECK(steer_weight_interpolation_->Init(xy4))
      << "Fail to load steer weight gain scheduler for MPC controller";
}

Status MPCController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    ControlCommand *cmd) {
  trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(planning_published_trajectory));

  SimpleMPCDebug *debug = cmd->mutable_debug()->mutable_simple_mpc_debug();
  debug->Clear();

  ComputeLongitudinalErrors(&trajectory_analyzer_, debug);

  // Update state
  UpdateState(debug);

  UpdateMatrix(debug);

  FeedforwardUpdate(debug);

  auto vehicle_state = injector_->vehicle_state();
  // Add gain scheduler for higher speed steering
  if (FLAGS_enable_gain_scheduler) {
    matrix_q_updated_(0, 0) =
        matrix_q_(0, 0) *
        lat_err_interpolation_->Interpolate(vehicle_state->linear_velocity());
    matrix_q_updated_(2, 2) =
        matrix_q_(2, 2) * heading_err_interpolation_->Interpolate(
                              vehicle_state->linear_velocity());
    steer_angle_feedforwardterm_updated_ =
        steer_angle_feedforwardterm_ *
        feedforwardterm_interpolation_->Interpolate(
            vehicle_state->linear_velocity());
    matrix_r_updated_(0, 0) =
        matrix_r_(0, 0) * steer_weight_interpolation_->Interpolate(
                              vehicle_state->linear_velocity());
  } else {
    matrix_q_updated_ = matrix_q_;
    matrix_r_updated_ = matrix_r_;
    steer_angle_feedforwardterm_updated_ = steer_angle_feedforwardterm_;
  }

  debug->add_matrix_q_updated(matrix_q_updated_(0, 0));
  debug->add_matrix_q_updated(matrix_q_updated_(1, 1));
  debug->add_matrix_q_updated(matrix_q_updated_(2, 2));
  debug->add_matrix_q_updated(matrix_q_updated_(3, 3));

  debug->add_matrix_r_updated(matrix_r_updated_(0, 0));
  debug->add_matrix_r_updated(matrix_r_updated_(1, 1));

  Matrix control_matrix = Matrix::Zero(controls_, 1);
  std::vector<Matrix> control(horizon_, control_matrix);

  Matrix control_gain_matrix = Matrix::Zero(controls_, basic_state_size_);
  std::vector<Matrix> control_gain(horizon_, control_gain_matrix);

  Matrix addition_gain_matrix = Matrix::Zero(controls_, 1);
  std::vector<Matrix> addition_gain(horizon_, addition_gain_matrix);

  Matrix reference_state = Matrix::Zero(basic_state_size_, 1);
  std::vector<Matrix> reference(horizon_, reference_state);

  Matrix lower_bound(controls_, 1);
  lower_bound << -wheel_single_direction_max_degree_, max_deceleration_;

  Matrix upper_bound(controls_, 1);
  upper_bound << wheel_single_direction_max_degree_, max_acceleration_;

  const double max = std::numeric_limits<double>::max();
  Matrix lower_state_bound(basic_state_size_, 1);
  Matrix upper_state_bound(basic_state_size_, 1);

  // lateral_error, lateral_error_rate, heading_error, heading_error_rate
  // station_error, station_error_rate
  lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max,
      -1.0 * max, -1.0 * max;
  upper_state_bound << max, max, M_PI, max, max, max;

  double mpc_start_timestamp = Clock::NowInSeconds();
  double steer_angle_feedback = 0.0;
  double acc_feedback = 0.0;
  double steer_angle_ff_compensation = 0.0;
  double unconstrained_control_diff = 0.0;
  double control_gain_truncation_ratio = 0.0;
  double unconstrained_control = 0.0;
  const double v = injector_->vehicle_state()->linear_velocity();

  std::vector<double> control_cmd(controls_, 0);

  apollo::common::math::MpcOsqp mpc_osqp(
      matrix_ad_, matrix_bd_, matrix_q_updated_, matrix_r_updated_,
      matrix_state_, lower_bound, upper_bound, lower_state_bound,
      upper_state_bound, reference_state, mpc_max_iteration_, horizon_,
      mpc_eps_);
  if (!mpc_osqp.Solve(&control_cmd)) {
    AERROR << "MPC OSQP solver failed";
  } else {
    ADEBUG << "MPC OSQP problem solved! ";
    control[0](0, 0) = control_cmd.at(0);
    control[0](1, 0) = control_cmd.at(1);
  }

  steer_angle_feedback = Wheel2SteerPct(control[0](0, 0));
  acc_feedback = control[0](1, 0);
  for (int i = 0; i < basic_state_size_; ++i) {
    unconstrained_control += control_gain[0](0, i) * matrix_state_(i, 0);
  }
  unconstrained_control += addition_gain[0](0, 0) * v * debug->curvature();
  if (enable_mpc_feedforward_compensation_) {
    unconstrained_control_diff =
        Wheel2SteerPct(control[0](0, 0) - unconstrained_control);
    if (fabs(unconstrained_control_diff) <= unconstrained_control_diff_limit_) {
      steer_angle_ff_compensation =
          Wheel2SteerPct(debug->curvature() *
                         (control_gain[0](0, 2) *
                              (lr_ - lf_ / cr_ * mass_ * v * v / wheelbase_) -
                          addition_gain[0](0, 0) * v));
    } else {
      control_gain_truncation_ratio = control[0](0, 0) / unconstrained_control;
      steer_angle_ff_compensation =
          Wheel2SteerPct(debug->curvature() *
                         (control_gain[0](0, 2) *
                              (lr_ - lf_ / cr_ * mass_ * v * v / wheelbase_) -
                          addition_gain[0](0, 0) * v) *
                         control_gain_truncation_ratio);
    }
  } else {
    steer_angle_ff_compensation = 0.0;
  }

  double mpc_end_timestamp = Clock::NowInSeconds();

  ADEBUG << "MPC core algorithm: calculation time is: "
         << (mpc_end_timestamp - mpc_start_timestamp) * 1000 << " ms.";

  // TODO(QiL): evaluate whether need to add spline smoothing after the result
  double steer_angle = steer_angle_feedback +
                       steer_angle_feedforwardterm_updated_ +
                       steer_angle_ff_compensation;

  if (FLAGS_set_steer_limit) {
    const double steer_limit = std::atan(max_lat_acc_ * wheelbase_ /
                                         (vehicle_state->linear_velocity() *
                                          vehicle_state->linear_velocity())) *
                               steer_ratio_ * 180 / M_PI /
                               steer_single_direction_max_degree_ * 100;

    // Clamp the steer angle with steer limitations at current speed
    double steer_angle_limited =
        common::math::Clamp(steer_angle, -steer_limit, steer_limit);
    steer_angle_limited = digital_filter_.Filter(steer_angle_limited);
    steer_angle = steer_angle_limited;
    debug->set_steer_angle_limited(steer_angle_limited);
  }
  steer_angle = digital_filter_.Filter(steer_angle);
  // Clamp the steer angle to -100.0 to 100.0
  steer_angle = common::math::Clamp(steer_angle, -100.0, 100.0);
  cmd->set_steering_target(steer_angle);

  debug->set_acceleration_cmd_closeloop(acc_feedback);

  double acceleration_cmd = acc_feedback + debug->acceleration_reference();
  // TODO(QiL): add pitch angle feed forward to accommodate for 3D control

  if ((planning_published_trajectory->trajectory_type() ==
       apollo::planning::ADCTrajectory::NORMAL) &&
      (std::fabs(debug->acceleration_reference()) <=
           max_acceleration_when_stopped_ &&
       std::fabs(debug->speed_reference()) <= max_abs_speed_when_stopped_)) {
    acceleration_cmd =
        (chassis->gear_location() == canbus::Chassis::GEAR_REVERSE)
            ? std::max(acceleration_cmd, -standstill_acceleration_)
            : std::min(acceleration_cmd, standstill_acceleration_);
    ADEBUG << "Stop location reached";
    debug->set_is_full_stop(true);
  }
  // TODO(Yu): study the necessity of path_remain and add it to MPC if needed

  debug->set_acceleration_cmd(acceleration_cmd);

  double calibration_value = 0.0;
  if (FLAGS_use_preview_speed_for_table) {
    calibration_value = control_interpolation_->Interpolate(
        std::make_pair(debug->speed_reference(), acceleration_cmd));
  } else {
    calibration_value = control_interpolation_->Interpolate(std::make_pair(
        injector_->vehicle_state()->linear_velocity(), acceleration_cmd));
  }

  debug->set_calibration_value(calibration_value);

  double throttle_cmd = 0.0;
  double brake_cmd = 0.0;
  if (calibration_value >= 0) {
    throttle_cmd = std::max(calibration_value, throttle_lowerbound_);
    brake_cmd = 0.0;
  } else {
    throttle_cmd = 0.0;
    brake_cmd = std::max(-calibration_value, brake_lowerbound_);
  }

  cmd->set_steering_rate(FLAGS_steer_angle_rate);
  // if the car is driven by acceleration, disgard the cmd->throttle and brake
  cmd->set_throttle(throttle_cmd);
  cmd->set_brake(brake_cmd);
  cmd->set_acceleration(acceleration_cmd);

  debug->set_heading(vehicle_state->heading());
  debug->set_steering_position(chassis->steering_percentage());
  debug->set_steer_angle(steer_angle);
  debug->set_steer_angle_feedforward(steer_angle_feedforwardterm_updated_);
  debug->set_steer_angle_feedforward_compensation(steer_angle_ff_compensation);
  debug->set_steer_unconstrained_control_diff(unconstrained_control_diff);
  debug->set_steer_angle_feedback(steer_angle_feedback);
  debug->set_steering_position(chassis->steering_percentage());

  if (std::fabs(vehicle_state->linear_velocity()) <=
          vehicle_param_.max_abs_speed_when_stopped() ||
      chassis->gear_location() == planning_published_trajectory->gear() ||
      chassis->gear_location() == canbus::Chassis::GEAR_NEUTRAL) {
    cmd->set_gear_location(planning_published_trajectory->gear());
  } else {
    cmd->set_gear_location(chassis->gear_location());
  }

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
  ADEBUG << "Control calibration table loaded";
  ADEBUG << "Control calibration table size is "
         << control_table.calibration_size();
  Interpolation2D::DataType xyz;
  for (const auto &calibration : control_table.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  control_interpolation_.reset(new Interpolation2D);
  ACHECK(control_interpolation_->Init(xyz))
      << "Fail to load control calibration table";
}

void MPCController::UpdateState(SimpleMPCDebug *debug) {
  const auto &com = injector_->vehicle_state()->ComputeCOMPosition(lr_);
  ComputeLateralErrors(com.x(), com.y(), injector_->vehicle_state()->heading(),
                       injector_->vehicle_state()->linear_velocity(),
                       injector_->vehicle_state()->angular_velocity(),
                       injector_->vehicle_state()->linear_acceleration(),
                       trajectory_analyzer_, debug);

  // State matrix update;
  matrix_state_(0, 0) = debug->lateral_error();
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(2, 0) = debug->heading_error();
  matrix_state_(3, 0) = debug->heading_error_rate();
  matrix_state_(4, 0) = debug->station_error();
  matrix_state_(5, 0) = debug->speed_error();
}

void MPCController::UpdateMatrix(SimpleMPCDebug *debug) {
  const double v = std::max(injector_->vehicle_state()->linear_velocity(),
                            minimum_speed_protection_);
  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;

  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
               (matrix_i + ts_ * 0.5 * matrix_a_);

  matrix_c_(1, 0) = (lr_ * cr_ - lf_ * cf_) / mass_ / v - v;
  matrix_c_(3, 0) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_ / v;
  matrix_cd_ = matrix_c_ * debug->ref_heading_rate() * ts_;
}

void MPCController::FeedforwardUpdate(SimpleMPCDebug *debug) {
  const double v = injector_->vehicle_state()->linear_velocity();
  const double kv =
      lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;
  steer_angle_feedforwardterm_ = Wheel2SteerPct(
      wheelbase_ * debug->curvature() + kv * v * v * debug->curvature());
}

void MPCController::ComputeLateralErrors(
    const double x, const double y, const double theta, const double linear_v,
    const double angular_v, const double linear_a,
    const TrajectoryAnalyzer &trajectory_analyzer, SimpleMPCDebug *debug) {
  const auto matched_point =
      trajectory_analyzer.QueryNearestPointByPosition(x, y);

  const double dx = x - matched_point.path_point().x();
  const double dy = y - matched_point.path_point().y();

  const double cos_matched_theta = std::cos(matched_point.path_point().theta());
  const double sin_matched_theta = std::sin(matched_point.path_point().theta());
  // d_error = cos_matched_theta * dy - sin_matched_theta * dx;
  debug->set_lateral_error(cos_matched_theta * dy - sin_matched_theta * dx);

  // matched_theta = matched_point.path_point().theta();
  debug->set_ref_heading(matched_point.path_point().theta());
  const double delta_theta =
      common::math::NormalizeAngle(theta - debug->ref_heading());
  debug->set_heading_error(delta_theta);

  const double sin_delta_theta = std::sin(delta_theta);
  // d_error_dot = chassis_v * sin_delta_theta;
  double lateral_error_dot = linear_v * sin_delta_theta;
  double lateral_error_dot_dot = linear_a * sin_delta_theta;
  if (FLAGS_reverse_heading_control) {
    if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
      lateral_error_dot = -lateral_error_dot;
      lateral_error_dot_dot = -lateral_error_dot_dot;
    }
  }

  debug->set_lateral_error_rate(lateral_error_dot);
  debug->set_lateral_acceleration(lateral_error_dot_dot);
  debug->set_lateral_jerk(
      (debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
  previous_lateral_acceleration_ = debug->lateral_acceleration();

  // matched_kappa = matched_point.path_point().kappa();
  debug->set_curvature(matched_point.path_point().kappa());
  // theta_error = delta_theta;
  debug->set_heading_error(delta_theta);
  // theta_error_dot = angular_v - matched_point.path_point().kappa() *
  // matched_point.v();
  debug->set_heading_rate(angular_v);
  debug->set_ref_heading_rate(debug->curvature() * matched_point.v());
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
}

void MPCController::ComputeLongitudinalErrors(
    const TrajectoryAnalyzer *trajectory_analyzer, SimpleMPCDebug *debug) {
  // the decomposed vehicle motion onto Frenet frame
  // s: longitudinal accumulated distance along reference trajectory
  // s_dot: longitudinal velocity along reference trajectory
  // d: lateral distance w.r.t. reference trajectory
  // d_dot: lateral distance change rate, i.e. dd/dt
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;

  const auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
      injector_->vehicle_state()->x(), injector_->vehicle_state()->y());

  trajectory_analyzer->ToTrajectoryFrame(
      injector_->vehicle_state()->x(), injector_->vehicle_state()->y(),
      injector_->vehicle_state()->heading(),
      injector_->vehicle_state()->linear_velocity(), matched_point, &s_matched,
      &s_dot_matched, &d_matched, &d_dot_matched);

  const double current_control_time = Clock::NowInSeconds();

  TrajectoryPoint reference_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(
          current_control_time);

  ADEBUG << "matched point:" << matched_point.DebugString();
  ADEBUG << "reference point:" << reference_point.DebugString();

  const double linear_v = injector_->vehicle_state()->linear_velocity();
  const double linear_a = injector_->vehicle_state()->linear_acceleration();
  double heading_error = common::math::NormalizeAngle(
      injector_->vehicle_state()->heading() - matched_point.theta());
  double lon_speed = linear_v * std::cos(heading_error);
  double lon_acceleration = linear_a * std::cos(heading_error);
  double one_minus_kappa_lat_error = 1 - reference_point.path_point().kappa() *
                                             linear_v * std::sin(heading_error);

  debug->set_station_reference(reference_point.path_point().s());
  debug->set_station_feedback(s_matched);
  debug->set_station_error(reference_point.path_point().s() - s_matched);
  debug->set_speed_reference(reference_point.v());
  debug->set_speed_feedback(lon_speed);
  debug->set_speed_error(reference_point.v() - s_dot_matched);
  debug->set_acceleration_reference(reference_point.a());
  debug->set_acceleration_feedback(lon_acceleration);
  debug->set_acceleration_error(reference_point.a() -
                                lon_acceleration / one_minus_kappa_lat_error);
  double jerk_reference =
      (debug->acceleration_reference() - previous_acceleration_reference_) /
      ts_;
  double lon_jerk =
      (debug->acceleration_feedback() - previous_acceleration_) / ts_;
  debug->set_jerk_reference(jerk_reference);
  debug->set_jerk_feedback(lon_jerk);
  debug->set_jerk_error(jerk_reference - lon_jerk / one_minus_kappa_lat_error);
  previous_acceleration_reference_ = debug->acceleration_reference();
  previous_acceleration_ = debug->acceleration_feedback();
}

}  // namespace control
}  // namespace apollo
