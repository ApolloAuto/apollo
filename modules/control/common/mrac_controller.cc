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

#include "modules/control/common/mrac_controller.h"

#include <cmath>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/LU"

#include "cyber/common/log.h"
#include "modules/common/util/string_util.h"

namespace apollo {
namespace control {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::util::StrCat;
using Matrix = Eigen::MatrixXd;

double MracController::Control(const double command, const Matrix state,
                               const double dt) {
  // check if the reference/adaption model well set up during the initilization
  if (!reference_model_enabled_ || !adaption_model_enabled_) {
    AWARN << "MRAC: model build failed; will work as a unity compensator. The "
             "reference_model building status: "
          << reference_model_enabled_
          << "; The adaption_model building status: "
          << adaption_model_enabled_;
    return command;  // treat the mrac as a unity proportional controller
  }
  // check if the current sampling time is valid
  if (dt <= 0.0) {
    AWARN << "MRAC: current sampling time <= 0, will use the last control, dt: "
          << dt;
    return control_previous_;
  }

  // update the state in the real actuation system
  state_action_.col(0) = state;

  // update the desired command in the real actuation system
  input_desired_(0, 0) = command;

  // update the state in the reference system
  Matrix matrix_i = Matrix::Identity(model_order_, model_order_);
  state_reference_.col(0) =
      (matrix_i - dt * 0.5 * matrix_a_reference_).inverse() *
      ((matrix_i + dt * 0.5 * matrix_a_reference_) * state_reference_.col(0) +
       dt * 0.5 * matrix_b_reference_ *
           (input_desired_(0, 0) + input_desired_(0, 1)));

  double state_bounded = 0.0;
  saturation_status_reference_ = BoundOutput(
      state_reference_(0, 0), state_reference_(0, 1), dt, &state_bounded);
  state_reference_(0, 0) = state_bounded;

  // update the adaption laws including state adaption, command adaption and
  // nonlinear components adaption
  Adaption(&gain_state_adaption_, state_action_,
           gamma_state_adaption_ * gamma_ratio_state_);
  Adaption(&gain_input_adaption_, input_desired_,
           gamma_input_adaption_ * gamma_ratio_input_);

  // update the generated control based on the adaptive law
  double control_unbounded =
      gain_state_adaption_.col(0).transpose() * state_action_.col(0) +
      gain_input_adaption_(0, 0) * input_desired_(0, 0);

  double control = 0.0;
  saturation_status_control_ =
      BoundOutput(control_unbounded, control_previous_, dt, &control);

  // update the anti-windup compensation if applied
  AntiWindupCompensation(control_unbounded, control_previous_, dt);

  // update the previous value for next iteration
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
  // reset all the externally-setting control parameters
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
  gain_input_adaption_.setZero(1, 2);
  gain_nonlinear_adaption_.setZero(1, 2);
}

void MracController::Init(const MracConf &mrac_conf, const double dt,
                          const double input_limit,
                          const double input_rate_limit) {
  control_previous_ = 0.0;
  saturation_status_control_ = 0;
  saturation_status_reference_ = 0;
  // Initialize the saturation limits
  bound_ratio_ = mrac_conf.mrac_saturation_level();
  bound_command_ = input_limit * bound_ratio_;
  bound_command_rate_ = input_rate_limit * bound_ratio_;
  // Initialize the common model parameters
  model_order_ = mrac_conf.mrac_reference_order();
  // Initialize the system states
  input_desired_ = Matrix::Zero(1, 2);
  state_action_ = Matrix::Zero(model_order_, 2);
  state_reference_ = Matrix::Zero(model_order_, 2);
  gain_state_adaption_ = Matrix::Zero(model_order_, 2);
  gain_input_adaption_ = Matrix::Zero(1, 2);
  gain_nonlinear_adaption_ = Matrix::Zero(1, 2);
  gamma_state_adaption_ = Matrix::Zero(model_order_, 1);
  gamma_input_adaption_ = Matrix::Zero(1, 1);
  gamma_nonlinear_adaption_ = Matrix::Zero(1, 1);
  compensation_anti_windup_ = Matrix::Zero(model_order_, 2);
  // Initialize the reference model parameters
  matrix_a_reference_ = Matrix::Zero(model_order_, model_order_);
  matrix_b_reference_ = Matrix::Zero(model_order_, 1);
  if (SetReferenceModel(mrac_conf).ok()) {
    BuildReferenceModel(dt);
  } else {
    reference_model_enabled_ = false;
  }
  // Initialize the adaption model parameters
  matrix_p_adaption_ = Matrix::Zero(model_order_, model_order_);
  matrix_b_adaption_ = Matrix::Zero(model_order_, 1);
  if (SetAdaptionModel(mrac_conf).ok()) {
    BuildAdaptionModel();
  } else {
    adaption_model_enabled_ = false;
  }
  // Initialize the anti-windup parameters
  gain_anti_windup_ = mrac_conf.anti_windup_compensation_gain();
}

Status MracController::SetReferenceModel(const MracConf &mrac_conf) {
  const double Epsilon = 0.000001;
  if (((mrac_conf.reference_time_constant() < Epsilon && model_order_ == 1)) ||
      ((mrac_conf.reference_natural_frequency() < Epsilon &&
        model_order_ == 2))) {
    const auto error_msg = StrCat(
        "mrac controller error: reference model time-constant parameter: ",
        mrac_conf.reference_time_constant(),
        "and natrual frequency parameter: ",
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
  return Status::OK();
}

Status MracController::SetAdaptionModel(const MracConf &mrac_conf) {
  if (mrac_conf.adaption_matrix_p_size() != matrix_p_adaption_.size()) {
    const auto error_msg =
        StrCat("mrac controller error: adaption matrix p element number: ",
               mrac_conf.adaption_matrix_p_size(),
               " in configuration file is not equal to desired matrix p size: ",
               matrix_p_adaption_.size());
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }
  for (int i = 0; i < model_order_; ++i) {
    gamma_state_adaption_(i, 0) = mrac_conf.adaption_state_gain();
    for (int j = 0; j < model_order_; ++j) {
      matrix_p_adaption_(i, j) =
          mrac_conf.adaption_matrix_p(i * model_order_ + j);
    }
  }
  gamma_input_adaption_(0, 0) = mrac_conf.adaption_desired_gain();
  gamma_nonlinear_adaption_(0, 0) = mrac_conf.adaption_nonlinear_gain();
  return Status::OK();
}

void MracController::BuildReferenceModel(const double dt) {
  reference_model_enabled_ = true;
  if (dt <= 0.0) {
    AWARN << "dt <= 0, continuous-discrete transformation for reference model "
             "failed, dt: "
          << dt;
    reference_model_enabled_ = false;
  } else {
    if (model_order_ == 1) {
      matrix_a_reference_(0, 0) = -1.0 / tau_reference_;
      matrix_b_reference_(0, 0) = 1.0 / tau_reference_;
    } else if (model_order_ == 2) {
      Ts_ = dt;
      matrix_a_reference_(0, 1) = 1.0;
      matrix_a_reference_(1, 0) = -wn_reference_ * wn_reference_;
      matrix_a_reference_(1, 1) = -2 * zeta_reference_ * wn_reference_;
      matrix_b_reference_(1, 0) = wn_reference_ * wn_reference_;
    } else {
      AWARN << "reference model order beyond the designed range, "
               "model_order: "
            << model_order_;
      reference_model_enabled_ = false;
    }
  }
}

void MracController::BuildAdaptionModel() {
  adaption_model_enabled_ = true;
  if (model_order_ <= 2) {
    if (model_order_ == 1) {
      matrix_b_adaption_(0, 0) = 1.0;
    } else {
      matrix_b_adaption_(1, 0) = 1.0;
    }
    if (!CheckLyapunovPD(matrix_a_reference_, matrix_p_adaption_)) {
      AWARN << "Solution of the algebraic Lyapunov equation is not symmetric "
               "positive definite";
      adaption_model_enabled_ = false;
    }
  } else {
    AWARN << "Adaption model order beyond the designed range, "
             "model_order: "
          << model_order_;
    adaption_model_enabled_ = false;
  }
}

bool MracController::CheckLyapunovPD(const Matrix matrix_a,
                                     const Matrix matrix_p) const {
  Matrix matrix_q = -matrix_p * matrix_a - matrix_a.transpose() * matrix_p;
  Eigen::LLT<Matrix> llt_matrix_q(matrix_q);
  // if matrix Q is not symmetric or the Cholkesky decomposition (LLT) failed
  // due to the matrix Q are not positive definite
  return (matrix_q.isApprox(matrix_q.transpose()) &&
          llt_matrix_q.info() != Eigen::NumericalIssue);
}

void MracController::Adaption(Matrix *law_adp, const Matrix state_adp,
                              const Matrix gain_adp) {
  Matrix state_error = state_action_ - state_reference_;
  law_adp->col(0) = law_adp->col(1) -
                    (0.5 * Ts_ * gain_adp * state_adp.col(0) *
                     (state_error.col(0) + compensation_anti_windup_.col(0))) -
                    (0.5 * Ts_ * gain_adp * state_adp.col(1) *
                     (state_error.col(1) + compensation_anti_windup_.col(1)));
  law_adp->col(1) = law_adp->col(0);
}

void MracController::AntiWindupCompensation(const double control_command,
                                            const double previous_command,
                                            const double dt) {
  Matrix offset_windup = Matrix::Zero(model_order_, 1);
  offset_windup(0, 0) =
      ((control_command > bound_command_) ? bound_command_ - control_command
                                          : 0.0) +
      ((control_command < -bound_command_) ? -bound_command_ - control_command
                                           : 0.0);
  if (model_order_ > 1) {
    offset_windup(1, 0) =
        ((control_command > previous_command + bound_command_rate_ * dt)
             ? bound_command_rate_ - (control_command - previous_command) / dt
             : 0.0) +
        ((control_command < previous_command - bound_command_rate_ * dt)
             ? -bound_command_rate_ - (control_command - previous_command) / dt
             : 0.0);
  }
  compensation_anti_windup_.col(1) = compensation_anti_windup_.col(0);
  compensation_anti_windup_.col(0) =
      gain_anti_windup_.transpose() * offset_windup;
}

int MracController::BoundOutput(const double output_unbounded,
                                const double previous_output, const double dt,
                                double *output) {
  int status = 0;
  if (output_unbounded > bound_command_ ||
      output_unbounded > previous_output + bound_command_rate_ * dt) {
    *output = (bound_command_ < previous_output + bound_command_rate_ * dt)
                  ? bound_command_
                  : previous_output + bound_command_rate_ * dt;
    // if output exceeds the upper bound, then status = 1; while if output
    // changing rate exceeds the upper rate bound, then status = 2
    status =
        (bound_command_ < previous_output + bound_command_rate_ * dt) ? 1 : 2;
  } else if (output_unbounded < -bound_command_ ||
             output_unbounded < previous_output - bound_command_rate_ * dt) {
    *output = (-bound_command_ > previous_output - bound_command_rate_ * dt)
                  ? -bound_command_
                  : previous_output - bound_command_rate_ * dt;
    // if output exceeds the lower bound, then status = -1; while if output
    // changing rate exceeds the lower rate bound, then status = -2
    status = (-bound_command_ > previous_output - bound_command_rate_ * dt)
                 ? -1
                 : -2;
  } else {
    *output = output_unbounded;
    // if output does not violate neithor bound nor rate bound, then status = 0
    status = 0;
  }
  return status;
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

double MracController::CurrentReferenceState() const {
  return state_reference_(0, 0);
}

double MracController::CurrentStateAdaptionGain() const {
  return gain_state_adaption_(0, 0);
}

double MracController::CurrentInputAdaptionGain() const {
  return gain_input_adaption_(0, 0);
}

}  // namespace control
}  // namespace apollo
