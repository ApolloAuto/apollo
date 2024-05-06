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

#include "modules/control/control_component/controller_task_base/common/mrac_controller.h"

#include <cmath>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/LU"
#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "modules/common/math/matrix_operations.h"

namespace apollo {
namespace control {

using apollo::common::ErrorCode;
using apollo::common::LatencyParam;
using apollo::common::Status;
using Matrix = Eigen::MatrixXd;

double MracController::Control(const double command, const Matrix state,
                               const double input_limit,
                               const double input_rate_limit) {
  // check if the current sampling time is valid and the reference/adaption
  // model well set up during the initialization
  if (ts_ <= 0.0 || !reference_model_enabled_ || !adaption_model_enabled_) {
    AERROR << "MRAC: model build failed; will work as a unity compensator. The "
              "reference_model building status: "
           << reference_model_enabled_
           << "; The adaption_model building status: "
           << adaption_model_enabled_ << "; Current sampling time: " << ts_;
    return command;  // treat the mrac as a unity proportional controller
  }

  // update the state in the real actuation system
  state_action_.col(0) = state;

  // update the desired command in the real actuation system
  input_desired_(0, 0) = command;

  // update the command bounds for the real actuation system
  bound_command_ = input_limit * bound_ratio_;
  bound_command_rate_ = input_rate_limit * bound_ratio_;

  // update the state in the reference system
  UpdateReference();

  double state_bounded = 0.0;
  saturation_status_reference_ = BoundOutput(
      state_reference_(0, 0), state_reference_(0, 1), &state_bounded);
  state_reference_(0, 0) = state_bounded;

  // update the adaption laws including state adaption, command adaption and
  // nonlinear components adaption
  UpdateAdaption(&gain_state_adaption_, state_action_,
                 gamma_state_adaption_ * gamma_ratio_state_);

  UpdateAdaption(&gain_input_adaption_, input_desired_,
                 gamma_input_adaption_ * gamma_ratio_input_);

  // revert the adaption law updates if they are beyond the clamping values
  if (model_order_ == 1 && adaption_clamping_enabled &&
      (gain_state_adaption_(0, 0) > 0.0 ||
       gain_state_adaption_(0, 0) < gain_state_clamping_(0, 0) ||
       gain_input_adaption_(0, 0) > gain_input_clamping_(0, 0) ||
       gain_input_adaption_(0, 0) < 1.0)) {
    gain_state_adaption_.col(0) = gain_state_adaption_.col(1);
    gain_input_adaption_.col(0) = gain_input_adaption_.col(1);
  }

  // update the generated control based on the adaptive law
  double control_unbounded =
      gain_state_adaption_.col(0).transpose() * state_action_.col(0) +
      gain_input_adaption_(0, 0) * input_desired_(0, 0);

  double control = 0.0;
  saturation_status_control_ =
      BoundOutput(control_unbounded, control_previous_, &control);

  // update the anti-windup compensation if applied
  AntiWindupCompensation(control_unbounded, control_previous_);

  // update the previous value for next iteration
  gain_state_adaption_.col(1) = gain_state_adaption_.col(0);
  gain_input_adaption_.col(1) = gain_input_adaption_.col(0);
  state_reference_.col(1) = state_reference_.col(0);
  state_action_.col(1) = state_action_.col(0);
  input_desired_.col(1) = input_desired_.col(0);
  control_previous_ = control;
  return control;
}

void MracController::Reset() {
  // reset the overall states
  ResetStates();
  // reset the adaptive gains
  ResetGains();
  // reset all the externally-setting, non-conf control parameters
  gamma_ratio_state_ = 1.0;
  gamma_ratio_input_ = 1.0;
  gamma_ratio_nonlinear_ = 1.0;
}

void MracController::ResetStates() {
  // reset the inputs and outputs of the closed-loop MRAC controller
  control_previous_ = 0.0;
  input_desired_.setZero(1, 2);
  // reset the internal states, anti-windup compensations and status
  state_action_.setZero(model_order_, 2);
  state_reference_.setZero(model_order_, 2);
  compensation_anti_windup_.setZero(model_order_, 2);
  saturation_status_reference_ = 0;
  saturation_status_control_ = 0;
}

void MracController::ResetGains() {
  gain_state_adaption_.setZero(model_order_, 2);
  gain_input_adaption_ = Matrix::Ones(1, 2);
  gain_nonlinear_adaption_.setZero(1, 2);
  gain_state_adaption_.col(1) = gain_state_adaption_init_;
  gain_input_adaption_.col(1) = gain_input_adaption_init_;
}

void MracController::Init(const MracConf &mrac_conf,
                          const LatencyParam &latency_param, const double dt) {
  control_previous_ = 0.0;
  saturation_status_control_ = 0;
  saturation_status_reference_ = 0;
  ts_ = dt;
  // Initialize the common model parameters
  model_order_ = mrac_conf.mrac_model_order();
  // Initialize the saturation limits
  bound_ratio_ = mrac_conf.mrac_saturation_level();
  // Initialize the system states
  input_desired_ = Matrix::Zero(1, 2);
  state_action_ = Matrix::Zero(model_order_, 2);
  state_reference_ = Matrix::Zero(model_order_, 2);
  // Initialize the adaptive control gains
  gain_state_adaption_ = Matrix::Zero(model_order_, 2);
  gain_input_adaption_ = Matrix::Ones(1, 2);
  gain_nonlinear_adaption_ = Matrix::Zero(1, 2);
  gain_state_clamping_ = Matrix::Zero(model_order_, 1);
  gain_input_clamping_ = Matrix::Ones(1, 1);
  gain_nonlinear_clamping_ = Matrix::Zero(1, 1);
  gain_state_adaption_init_ = Matrix::Zero(model_order_, 1);
  gain_input_adaption_init_ = Matrix::Ones(1, 1);
  gain_nonlinear_adaption_init_ = Matrix::Zero(1, 1);
  // Initialize the adaptive convergence gains and anti-windup gains
  gamma_state_adaption_ = Matrix::Zero(model_order_, model_order_);
  gamma_input_adaption_ = Matrix::Zero(1, 1);
  gamma_nonlinear_adaption_ = Matrix::Zero(1, 1);
  gain_anti_windup_ = Matrix::Zero(model_order_, model_order_);
  compensation_anti_windup_ = Matrix::Zero(model_order_, 2);
  // Initialize the reference model parameters
  matrix_a_reference_ = Matrix::Zero(model_order_, model_order_);
  matrix_b_reference_ = Matrix::Zero(model_order_, 1);
  reference_model_enabled_ =
      (SetReferenceModel(mrac_conf).ok() && BuildReferenceModel().ok());
  // Initialize the adaption model parameters
  matrix_p_adaption_ = Matrix::Zero(model_order_, model_order_);
  matrix_b_adaption_ = Matrix::Zero(model_order_, 1);
  adaption_model_enabled_ =
      (SetAdaptionModel(mrac_conf).ok() && BuildAdaptionModel().ok());
  EstimateInitialGains(latency_param);
  gain_state_adaption_.col(1) = gain_state_adaption_init_;
  gain_input_adaption_.col(1) = gain_input_adaption_init_;
}

Status MracController::SetReferenceModel(const MracConf &mrac_conf) {
  const double Epsilon = 0.000001;
  if (((mrac_conf.reference_time_constant() < Epsilon && model_order_ == 1)) ||
      ((mrac_conf.reference_natural_frequency() < Epsilon &&
        model_order_ == 2))) {
    const auto error_msg = absl::StrCat(
        "mrac controller error: reference model time-constant parameter: ",
        mrac_conf.reference_time_constant(),
        "and natural frequency parameter: ",
        mrac_conf.reference_natural_frequency(),
        " in configuration file are not reasonable with respect to the "
        "reference model order: ",
        model_order_);
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }
  tau_reference_ = mrac_conf.reference_time_constant();
  wn_reference_ = mrac_conf.reference_natural_frequency();
  zeta_reference_ = mrac_conf.reference_damping_ratio();
  tau_clamping_ = mrac_conf.clamping_time_constant();
  adaption_clamping_enabled = (tau_clamping_ >= Epsilon);
  return Status::OK();
}

Status MracController::SetAdaptionModel(const MracConf &mrac_conf) {
  const int p_size = mrac_conf.adaption_matrix_p_size();
  const int x_size = mrac_conf.adaption_state_gain_size();
  const int aw_size = mrac_conf.anti_windup_compensation_gain_size();
  if (p_size != model_order_ * model_order_ || x_size > model_order_ ||
      aw_size > model_order_) {
    const auto error_msg = absl::StrCat(
        "mrac controller error: adaption matrix p element number: ", p_size,
        ", state gain number: ", x_size,
        ", and anti-windup compensation gain number: ", aw_size,
        " in configuration file do not match the model number: ", model_order_);
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }
  for (int i = 0; i < model_order_; ++i) {
    gamma_state_adaption_(i, i) =
        (i <= x_size - 1) ? mrac_conf.adaption_state_gain(i)
                          : mrac_conf.adaption_state_gain(x_size - 1);
    gain_anti_windup_(i, i) =
        (i <= aw_size - 1)
            ? mrac_conf.anti_windup_compensation_gain(i)
            : mrac_conf.anti_windup_compensation_gain(aw_size - 1);
    for (int j = 0; j < model_order_; ++j) {
      matrix_p_adaption_(i, j) =
          mrac_conf.adaption_matrix_p(i * model_order_ + j);
    }
  }
  gamma_input_adaption_(0, 0) = mrac_conf.adaption_desired_gain();
  gamma_nonlinear_adaption_(0, 0) = mrac_conf.adaption_nonlinear_gain();
  return Status::OK();
}

Status MracController::BuildReferenceModel() {
  if (model_order_ > 2) {
    const auto error_msg =
        absl::StrCat("mrac controller error: reference model order ",
                     model_order_, " is beyond the designed range");
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }
  if (model_order_ == 1) {
    matrix_a_reference_(0, 0) = -1.0 / tau_reference_;
    matrix_b_reference_(0, 0) = 1.0 / tau_reference_;
  } else if (model_order_ == 2) {
    matrix_a_reference_(0, 1) = 1.0;
    matrix_a_reference_(1, 0) = -wn_reference_ * wn_reference_;
    matrix_a_reference_(1, 1) = -2 * zeta_reference_ * wn_reference_;
    matrix_b_reference_(1, 0) = wn_reference_ * wn_reference_;
  }
  return Status::OK();
}

Status MracController::BuildAdaptionModel() {
  if (model_order_ > 2) {
    const auto error_msg =
        absl::StrCat("mrac controller error: adaption model order ",
                     model_order_, " is beyond the designed range");
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }
  if (model_order_ == 1) {
    matrix_b_adaption_(0, 0) = 1.0;
  } else if (model_order_ == 2) {
    matrix_b_adaption_(1, 0) = wn_reference_ * wn_reference_;
  }
  if (!CheckLyapunovPD(matrix_a_reference_, matrix_p_adaption_)) {
    const std::string error_msg =
        "Solution of the algebraic Lyapunov equation is not symmetric positive "
        "definite";
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }
  return Status::OK();
}

bool MracController::CheckLyapunovPD(const Matrix matrix_a,
                                     const Matrix matrix_p) const {
  Matrix matrix_q = -matrix_p * matrix_a - matrix_a.transpose() * matrix_p;
  Eigen::LLT<Matrix> llt_matrix_q(matrix_q);
  // if matrix Q is not symmetric or the Cholesky decomposition (LLT) failed
  // due to the matrix Q are not positive definite
  return (matrix_q.isApprox(matrix_q.transpose()) &&
          llt_matrix_q.info() != Eigen::NumericalIssue);
}

void MracController::EstimateInitialGains(const LatencyParam &latency_param) {
  const double rise_time_estimate =
      latency_param.dead_time() + latency_param.rise_time();
  const double settling_time_estimate =
      latency_param.dead_time() + latency_param.settling_time();
  Matrix matrix_a_estimate = Matrix::Zero(model_order_, model_order_);
  Matrix matrix_b_estimate = Matrix::Zero(model_order_, 1);
  Matrix matrix_a_clamping = Matrix::Zero(model_order_, model_order_);
  Matrix matrix_b_clamping = Matrix::Zero(model_order_, 1);
  if (model_order_ == 1 &&
      (rise_time_estimate >= ts_ || settling_time_estimate >= ts_)) {
    const double tau_estimate = (rise_time_estimate >= ts_)
                                    ? rise_time_estimate / 2.2
                                    : settling_time_estimate / 4.0;
    // generate the initial adaptive gains
    matrix_a_estimate(0, 0) = -1.0 / tau_estimate;
    matrix_b_estimate(0, 0) = 1.0 / tau_estimate;
    gain_state_adaption_init_(0, 0) =
        (matrix_a_reference_(0, 0) - matrix_a_estimate(0, 0)) /
        matrix_b_estimate(0, 0);
    gain_input_adaption_init_(0, 0) =
        matrix_b_reference_(0, 0) / matrix_b_estimate(0, 0);
    // generate the clamping bounds for adaptive gains
    if (adaption_clamping_enabled) {
      matrix_a_clamping(0, 0) = -1.0 / tau_clamping_;
      matrix_b_clamping(0, 0) = 1.0 / tau_clamping_;
      gain_state_clamping_(0, 0) =
          (matrix_a_clamping(0, 0) - matrix_a_estimate(0, 0)) /
          matrix_b_estimate(0, 0);
      gain_input_clamping_(0, 0) =
          matrix_b_clamping(0, 0) / matrix_b_estimate(0, 0);
    }
  } else if (model_order_ == 2 &&
             (rise_time_estimate >= ts_ && settling_time_estimate >= ts_)) {
    const double wn_estimate = 1.8 / rise_time_estimate;
    const double zeta_estimate =
        4.6 / (rise_time_estimate * settling_time_estimate);
    matrix_a_estimate(0, 1) = 1.0;
    matrix_a_estimate(1, 0) = -wn_estimate * wn_estimate;
    matrix_a_estimate(1, 1) = -2 * zeta_estimate * wn_estimate;
    matrix_b_estimate(1, 0) = wn_estimate * wn_estimate;
    gain_state_adaption_init_.col(0) =
        (common::math::PseudoInverse<double, 2, 1>(matrix_b_estimate) *
         (matrix_a_reference_ - matrix_a_estimate))
            .transpose();
    gain_input_adaption_init_.col(0) =
        (common::math::PseudoInverse<double, 2, 1>(matrix_b_estimate) *
         matrix_b_reference_)
            .transpose();
  } else {
    AWARN << "No pre-known actuation dynamics; the initial states of the "
             "adaptive gains are set as zeros";
  }
}

void MracController::UpdateReference() {
  Matrix matrix_i = Matrix::Identity(model_order_, model_order_);
  state_reference_.col(0) =
      (matrix_i - ts_ * 0.5 * matrix_a_reference_).inverse() *
      ((matrix_i + ts_ * 0.5 * matrix_a_reference_) * state_reference_.col(1) +
       ts_ * 0.5 * matrix_b_reference_ *
           (input_desired_(0, 0) + input_desired_(0, 1)));
}

void MracController::UpdateAdaption(Matrix *law_adp, const Matrix state_adp,
                                    const Matrix gain_adp) {
  Matrix state_error = state_action_ - state_reference_;
  law_adp->col(0) =
      law_adp->col(1) -
      0.5 * ts_ * gain_adp *
          (state_adp.col(0) * (state_error.col(0).transpose() +
                               compensation_anti_windup_.col(0).transpose()) +
           state_adp.col(1) * (state_error.col(1).transpose() +
                               compensation_anti_windup_.col(1).transpose())) *
          matrix_p_adaption_ * matrix_b_adaption_;
}

void MracController::AntiWindupCompensation(const double control_command,
                                            const double previous_command) {
  Matrix offset_windup = Matrix::Zero(model_order_, 1);
  offset_windup(0, 0) =
      ((control_command > bound_command_) ? bound_command_ - control_command
                                          : 0.0) +
      ((control_command < -bound_command_) ? -bound_command_ - control_command
                                           : 0.0);
  if (model_order_ > 1) {
    offset_windup(1, 0) =
        ((control_command > previous_command + bound_command_rate_ * ts_)
             ? bound_command_rate_ - (control_command - previous_command) / ts_
             : 0.0) +
        ((control_command < previous_command - bound_command_rate_ * ts_)
             ? -bound_command_rate_ - (control_command - previous_command) / ts_
             : 0.0);
  }
  compensation_anti_windup_.col(1) = compensation_anti_windup_.col(0);
  compensation_anti_windup_.col(0) = gain_anti_windup_ * offset_windup;
}

int MracController::BoundOutput(const double output_unbounded,
                                const double previous_output, double *output) {
  int status = 0;
  if (output_unbounded > bound_command_ ||
      output_unbounded > previous_output + bound_command_rate_ * ts_) {
    *output = (bound_command_ < previous_output + bound_command_rate_ * ts_)
                  ? bound_command_
                  : previous_output + bound_command_rate_ * ts_;
    // if output exceeds the upper bound, then status = 1; while if output
    // changing rate exceeds the upper rate bound, then status = 2
    status =
        (bound_command_ < previous_output + bound_command_rate_ * ts_) ? 1 : 2;
  } else if (output_unbounded < -bound_command_ ||
             output_unbounded < previous_output - bound_command_rate_ * ts_) {
    *output = (-bound_command_ > previous_output - bound_command_rate_ * ts_)
                  ? -bound_command_
                  : previous_output - bound_command_rate_ * ts_;
    // if output exceeds the lower bound, then status = -1; while if output
    // changing rate exceeds the lower rate bound, then status = -2
    status = (-bound_command_ > previous_output - bound_command_rate_ * ts_)
                 ? -1
                 : -2;
  } else {
    *output = output_unbounded;
    // if output does not violate neither bound nor rate bound, then status = 0
    status = 0;
  }
  return status;
}

void MracController::SetInitialReferenceState(
    const Matrix &state_reference_init) {
  if (state_reference_init.rows() != model_order_ ||
      state_reference_init.cols() != 1) {
    AWARN << "failed to set the initial reference states, due to the given "
             "state size: "
          << state_reference_init.rows() << " x " << state_reference_init.cols()
          << " doesn't match the model order: " << model_order_;
  } else {
    state_reference_.col(1) = state_reference_init;
  }
}

void MracController::SetInitialActionState(const Matrix &state_action_init) {
  if (state_action_init.rows() != model_order_ ||
      state_action_init.cols() != 1) {
    AWARN << "failed to set the initial action states, due to the given "
             "state size: "
          << state_action_init.rows() << " x " << state_action_init.cols()
          << " doesn't match the model order: " << model_order_;
  } else {
    state_action_.col(1) = state_action_init;
  }
}

void MracController::SetInitialCommand(const double command_init) {
  input_desired_(0, 1) = command_init;
}

void MracController::SetInitialStateAdaptionGain(
    const Matrix &gain_state_adaption_init) {
  if (gain_state_adaption_init.rows() != model_order_ ||
      gain_state_adaption_init.cols() != 1) {
    AWARN << "failed to set the initial state adaption gains, due to the given "
             "state size: "
          << gain_state_adaption_init.rows() << " x "
          << gain_state_adaption_init.cols()
          << " doesn't match the model order: " << model_order_;
  } else {
    gain_state_adaption_.col(1) = gain_state_adaption_init;
  }
}

void MracController::SetInitialInputAdaptionGain(
    const double gain_input_adaption_init) {
  gain_input_adaption_(0, 1) = gain_input_adaption_init;
}

void MracController::SetInitialNonlinearAdaptionGain(
    const double gain_nonlinear_adaption_init) {
  gain_nonlinear_adaption_(0, 1) = gain_nonlinear_adaption_init;
}

void MracController::SetStateAdaptionRate(const double ratio_state) {
  if (ratio_state < 0.0) {
    AWARN << "failed to set the state adaption rate, due to new ratio < 0; the "
             "current ratio is still: "
          << gamma_ratio_state_;
  } else {
    gamma_ratio_state_ = ratio_state;
  }
}

void MracController::SetInputAdaptionRate(const double ratio_input) {
  if (ratio_input < 0.0) {
    AWARN << "failed to set the input adaption rate, due to new ratio < 0; the "
             "current ratio is still: "
          << gamma_ratio_input_;
  } else {
    gamma_ratio_input_ = ratio_input;
  }
}

void MracController::SetNonlinearAdaptionRate(const double ratio_nonlinear) {
  if (ratio_nonlinear < 0.0) {
    AWARN << "failed to set the nonlinear adaption rate, due to new ratio < 0; "
             "the current ratio is still: "
          << gamma_ratio_nonlinear_;
  } else {
    gamma_ratio_nonlinear_ = ratio_nonlinear;
  }
}

double MracController::StateAdaptionRate() const { return gamma_ratio_state_; }

double MracController::InputAdaptionRate() const { return gamma_ratio_input_; }

double MracController::NonlinearAdaptionRate() const {
  return gamma_ratio_nonlinear_;
}

int MracController::ReferenceSaturationStatus() const {
  return saturation_status_reference_;
}

int MracController::ControlSaturationStatus() const {
  return saturation_status_control_;
}

Matrix MracController::CurrentReferenceState() const {
  return state_reference_;
}

Matrix MracController::CurrentStateAdaptionGain() const {
  return gain_state_adaption_;
}

Matrix MracController::CurrentInputAdaptionGain() const {
  return gain_input_adaption_;
}

Matrix MracController::CurrentNonlinearAdaptionGain() const {
  return gain_nonlinear_adaption_;
}

}  // namespace control
}  // namespace apollo
